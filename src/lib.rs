#[cfg(target_os = "linux")]
// mod hexmove;
mod hiwonder;
mod inspirehand;
mod process_manager;
mod proxyactuator;
mod rh56actuator;
mod rsactuator;
pub use proxyactuator::*;
pub use rh56actuator::*;
pub use robstride::{ActuatorConfiguration, ActuatorType};
pub use rsactuator::*;

#[cfg(target_os = "linux")]
// pub use hexmove::*;
pub use hiwonder::*;
pub use inspirehand::*;
pub use process_manager::*;

use async_trait::async_trait;
use eyre::eyre;
use eyre::WrapErr;
use kbot_pwrbrd::PowerBoard;
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

pub struct KbotPlatform {}

impl KbotPlatform {
    pub fn new() -> Self {
        Self {}
    }

    fn initialize_powerboard(&self) -> eyre::Result<()> {
        let board = PowerBoard::new("can0")
        .map_err(|e| eyre!("Failed to initialize power board: {}", e))?;

        Spawn power monitoring loop
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            loop {
                interval.tick().await;

                let data = match board.query_data() {
                    Ok(data) => data,
                    Err(e) => {
                        tracing::error!("Error querying power board: {}", e.to_string());
                        continue;
                    }
                };

                let telemetry = Telemetry::get().await;
                if let Some(telemetry) = telemetry {
                    if let Err(e) = telemetry.publish("powerboard/data", &data).await {
                        tracing::error!("Failed to publish power board data: {:?}", e);
                    }
                }
                tracing::trace!("Power board data: {:?}", data);
            }
        });

        Ok(())
    }
}

impl Default for KbotPlatform {
    fn default() -> Self {
        KbotPlatform::new()
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

                // Create RH56 actuator for hand control (IDs 51-56)
                let rh56 =
                    RH56Actuator::new(operations_service.clone(), "/dev/ttyUSB1", 1, 51).await?;

                // Create RobStride actuator for robot joints (IDs 11-45)
                let rs = RSActuator::new(
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
                        (
                            16,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride00,
                                max_angle_change: Some(50.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
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
                        (
                            26,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride00,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
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
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            33,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride03,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            34,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            35,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
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
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            44,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride04,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                        (
                            45,
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride02,
                                max_angle_change: Some(30.0f32.to_radians()),
                                max_velocity: Some(10.0f32.to_radians()),
                            },
                        ),
                    ],
                )
                .await
                .wrap_err("Failed to create actuator")?;

                // Create proxy actuator that combines both
                let proxy = ProxyActuator::new(vec![
                    (Box::new(rh56), 51..=56), // Hand control
                    (Box::new(rs), 11..=45),   // Robot joints
                ]);

                Ok(vec![
                    // ServiceEnum::Imu(ImuServiceServer::new(IMUServiceImpl::new(Arc::new(imu)))),
                    ServiceEnum::Actuator(ActuatorServiceServer::new(ActuatorServiceImpl::new(
                        Arc::new(proxy),
                    ))),
                    ServiceEnum::ProcessManager(ProcessManagerServiceServer::new(
                        ProcessManagerServiceImpl::new(Arc::new(process_manager)),
                    )),
                ])
            } else {
                let actuator = RSActuator::new(
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
        // Shutdown and cleanup code goes here
        Ok(())
    }
}
