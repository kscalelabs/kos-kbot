mod actuator;
mod process_manager;

#[cfg(target_os = "linux")]
// mod hexmove;
mod hiwonder;

pub use actuator::*;
pub use robstride::{ActuatorConfiguration, ActuatorType};

#[cfg(target_os = "linux")]
// pub use hexmove::*;
pub use hiwonder::*;
pub use process_manager::*;

use async_trait::async_trait;
use eyre::eyre;
use eyre::WrapErr;
use kbot_pwrbrd::{PowerBoard, PowerBoardFrame};
use kos::hal::Operation;
use kos::kos_proto::actuator::actuator_service_server::ActuatorServiceServer;
use kos::kos_proto::imu::imu_service_server::ImuServiceServer;
use kos::kos_proto::process_manager::process_manager_service_server::ProcessManagerServiceServer;
use kos::{
    services::{
        ActuatorServiceImpl, IMUServiceImpl, OperationsServiceImpl, ProcessManagerServiceImpl,
    },
    telemetry::Telemetry,
    Platform, ServiceEnum,
};
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use serde_json;
use once_cell::sync::OnceCell;
use parking_lot;

pub struct KbotPlatform {}

impl KbotPlatform {
    pub fn new() -> Self {
        Self {}
    }

    fn initialize_powerboard(&self) -> eyre::Result<()> {
        let board = PowerBoard::new("can0")
            .map_err(|e| eyre!("Failed to initialize power board: {}", e))?;

        tracing::info!("Initializing power board monitoring on can0");
        board.configure_board().map_err(|e| eyre!("Failed to configure power board: {}", e))?;

        // Create a shutdown channel
        let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
        
        // Store sender in a static or global location for shutdown
        SHUTDOWN_SIGNAL.get_or_init(|| parking_lot::Mutex::new(Some(shutdown_tx)));

        tokio::task::spawn_blocking(move || {
            let rt = tokio::runtime::Runtime::new()
                .expect("Failed to create runtime for power board monitoring");
            
            rt.block_on(async {
                let mut counter = 0u64;
                let mut interval = tokio::time::interval(Duration::from_millis(100));
                let mut shutdown_rx = shutdown_rx;

                loop {
                    tokio::select! {
                        _ = interval.tick() => {
                            counter = counter.wrapping_add(1);

                            if let Ok(Some(frame)) = board.read_frame() {
                                let telemetry = Telemetry::get().await;
                                
                                match frame {
                                    PowerBoardFrame::General(status) => {                                        
                                        if let Some(telemetry) = &telemetry {
                                            let data = serde_json::json!({
                                                "counter": counter,
                                                "data": status,
                                            });
                                            if let Err(e) = telemetry.publish("powerboard/general", &data).await {
                                                tracing::error!("Failed to publish power board general data: {:?}", e);
                                            }
                                        }

                                        tracing::trace!(
                                            "General[{}]: {:.2}V {:.2}A",
                                            counter,
                                            status.battery_voltage,
                                            status.current,
                                        );
                                    }
                                    PowerBoardFrame::Limbs(status) => {
                                        
                                        if let Some(telemetry) = &telemetry {
                                            let data = serde_json::json!({
                                                "counter": counter,
                                                "data": status,
                                            });
                                            if let Err(e) = telemetry.publish("powerboard/limbs", &data).await {
                                                tracing::error!("Failed to publish power board limbs data: {:?}", e);
                                            }
                                        }

                                        tracing::trace!(
                                            "Limbs[{}]: L:{:.1}W R:{:.1}W LA:{:.1}W RA:{:.1}W",
                                            counter,
                                            status.left_leg_power,
                                            status.right_leg_power,
                                            status.left_arm_power,
                                            status.right_arm_power,
                                        );
                                    }
                                    PowerBoardFrame::Unknown(_, _) => {}
                                }
                            }else{
                                tracing::error!("Failed to read power board frame");
                            }
                        }
                        Ok(_) = shutdown_rx.changed() => {
                            tracing::info!("Shutting down power board monitoring");
                            break;
                        }
                    }
                }
            });
        });

        Ok(())
    }
}

impl Default for KbotPlatform {
    fn default() -> Self {
        KbotPlatform::new()
    }
}

impl Drop for KbotPlatform {
    fn drop(&mut self) {
        // Ensure shutdown is called when the platform is dropped
        let _ = self.shutdown();
    }
}

#[async_trait]
impl Platform for KbotPlatform {
    fn name(&self) -> &'static str {
        "KBot"
    }

    fn serial(&self) -> String {
        // TODO: Get the serial number from the device
        "00000000".to_string()
    }

    fn initialize(&mut self, _operations_service: Arc<OperationsServiceImpl>) -> eyre::Result<()> {
        // Initialize the platform
        self.initialize_powerboard()?;
        Ok(())
    }

    fn create_services<'a>(
        &'a self,
        operations_service: Arc<OperationsServiceImpl>,
    ) -> Pin<Box<dyn Future<Output = eyre::Result<Vec<ServiceEnum>>> + Send + 'a>> {
        Box::pin(async move {
            if cfg!(target_os = "linux") {
                tracing::debug!("Initializing KBot services for Linux");

                let process_manager =
                    KBotProcessManager::new(self.name().to_string(), self.serial())
                        .wrap_err("Failed to initialize GStreamer process manager")?;

                let actuator = KBotActuator::new(
                    operations_service.clone(),
                    vec![
                        // "/dev/ttyCH341USB0",
                        // "/dev/ttyCH341USB1",
                        // "/dev/ttyCH341USB2",
                        // "/dev/ttyCH341USB3",
                        // "can0",
                        "can1", "can2", "can3", "can4",
                    ],
                    Duration::from_secs(1),
                    // Duration::from_nanos(3_333_333),
                    Duration::from_millis(7),
                    &[
                        // Left Arm
                        (
                            11,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            12,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            13,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            14,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            15,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        // (
                        //     16,
                        //     ActuatorConfiguration {
                        //         actuator_type: ActuatorType::RobStride00,
                        //         max_angle_change: Some(15.0f32.to_radians()),
                        //         max_velocity: Some(10.0f32.to_radians()),
                        //     },
                        // ),
                        // Right Arm
                        (
                            21,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            22,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            23,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            24,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            25,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        // (
                        //     26,
                        //     ActuatorConfiguration {
                        //         actuator_type: ActuatorType::RobStride00,
                        //         max_angle_change: Some(15.0f32.to_radians()),
                        //         max_velocity: Some(10.0f32.to_radians()),
                        //     },
                        // ),
                        // Left Leg
                        (
                            31,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            32,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(45.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            33,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(90.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            34,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(45.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            35,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(90.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        // Right Leg
                        (
                            41,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            42,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            43,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(90.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            44,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(45.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            45,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(90.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                    ],
                )
                .await
                .wrap_err("Failed to create actuator")?;

                // let imu = KBotIMU::new(operations_service.clone(), "/dev/ttyCH341USB0", 9600)
                // let imu = KBotIMU::new(operations_service.clone(), "/dev/ttyCH341USB1", 9600)
                let imu = KBotIMU::new(operations_service.clone(), "/dev/ttyUSB0", 9600)
                    .wrap_err("Failed to create IMU")?;

                Ok(vec![
                    ServiceEnum::Imu(ImuServiceServer::new(IMUServiceImpl::new(Arc::new(imu)))),
                    ServiceEnum::Actuator(ActuatorServiceServer::new(ActuatorServiceImpl::new(
                        Arc::new(actuator),
                    ))),
                    ServiceEnum::ProcessManager(ProcessManagerServiceServer::new(
                        ProcessManagerServiceImpl::new(Arc::new(process_manager)),
                    )),
                ])
            } else {
                let actuator = KBotActuator::new(
                    operations_service,
                    vec!["can0"],
                    Duration::from_secs(1),
                    Duration::from_nanos(3_333_333),
                    &[(
                        1,
                        robstride::ActuatorConfiguration {
                            actuator_type: ActuatorType::RobStride04,
                            max_angle_change: Some(2.0f32.to_radians()),
                            max_velocity: Some(10.0f32.to_radians()),
                        },
                    )],
                )
                .await
                .wrap_err("Failed to create actuator")?;

                Ok(vec![ServiceEnum::Actuator(ActuatorServiceServer::new(
                    ActuatorServiceImpl::new(Arc::new(actuator)),
                ))])
            }
        })
    }

    fn shutdown(&mut self) -> eyre::Result<()> {
        // Signal powerboard monitoring to stop
        if let Some(shutdown) = SHUTDOWN_SIGNAL.get().and_then(|lock| lock.lock().take()) {
            tracing::info!("Sending shutdown signal to power board monitoring");
            let _ = shutdown.send(true);
        }
        Ok(())
    }
}

static SHUTDOWN_SIGNAL: OnceCell<parking_lot::Mutex<Option<tokio::sync::watch::Sender<bool>>>> = OnceCell::new();
