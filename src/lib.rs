mod inspirehand;
mod process_manager;
mod rh56actuator;
mod rsactuator;
mod proxyactuator;
#[cfg(target_os = "linux")]
// mod hexmove;
mod hiwonder;

pub use inspirehand::*;
pub use rh56actuator::*;
pub use rsactuator::*;
pub use proxyactuator::*;
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
use once_cell::sync::OnceCell;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

const USE_POWERBOARD: bool = false;
const USE_HANDS: bool = false;

pub struct KbotPlatform {}

impl KbotPlatform {
    pub fn new() -> Self {
        Self {}
    }

    fn initialize_powerboard(&self) -> eyre::Result<()> {
        let board = PowerBoard::new("can2")
            .map_err(|e| eyre!("Failed to initialize power board: {}", e))?;

        tracing::info!("Initializing power board monitoring on can0");
        board
            .configure_board()
            .map_err(|e| eyre!("Failed to configure power board: {}", e))?;

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
        if USE_POWERBOARD {
            self.initialize_powerboard()?;
        }
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

                let mut services: Vec<ServiceEnum> = Vec::new();

                // Initialize Actuator
                let max_vel = 7200.0f32.to_radians();

                let rs_actuator = RSActuator::new(
                    operations_service.clone(),
                    vec![
                        // "/dev/ttyCH341USB0",
                        // "/dev/ttyCH341USB1",
                        // "/dev/ttyCH341USB2",
                        // "/dev/ttyCH341USB3",
                        // "can0",
                        "can1", "can0", "can2", "can3", "can4",
                    ],
                    Duration::from_secs(1),
                    Duration::from_millis(2),
                    &[
                        // Left Arm
                        (
                            11,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            12,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            13,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(45.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            14,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(55.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            15,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(60.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
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
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            22,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            23,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(45.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            24,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(55.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            25,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(60.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
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
                                max_angle_change: Some(2.0 * 30.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            32,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(2.0 * 45.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            33,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(2.0 * 90.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            34,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(2.0 * 45.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            35,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(2.0 * 90.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        // Right Leg
                        (
                            41,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(2.0 * 30.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            42,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(2.0 * 45.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            43,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(2.0 * 90.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            44,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(2.0 * 45.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                        (
                            45,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(2.0 * 90.0f32.to_radians()),
                                max_velocity: Some(max_vel),
                                command_rate_hz: Some(100.0),
                            },
                        ),
                    ],
                )
                .await
                .wrap_err("Failed to create actuator")?;
                
                if USE_HANDS {
                    let left_hand = RH56Actuator::new(
                        operations_service.clone(),
                        "/dev/ttyUSB0",
                        1,  
                        51,
                    )
                    .await
                    .wrap_err("Failed to create left hand")?;
    
                    let right_hand = RH56Actuator::new(
                        operations_service.clone(),
                        "/dev/ttyUSB1",
                        1,
                        61,
                    )
                    .await
                    .wrap_err("Failed to create right hand")?;
                }


                actuators_to_add = vec![
                    (Box::new(rs_actuator), 1..=49),
                ]

                if USE_HANDS {
                    actuators_to_add.push((Box::new(left_hand), 51..=56));
                    actuators_to_add.push((Box::new(right_hand), 61..=66));
                }

                let actuator = ProxyActuator::new(actuators_to_add);
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
                unimplemented!("ouch");
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

static SHUTDOWN_SIGNAL: OnceCell<parking_lot::Mutex<Option<tokio::sync::watch::Sender<bool>>>> =
    OnceCell::new();
