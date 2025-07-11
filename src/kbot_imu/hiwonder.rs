use kos::{
    hal::{
        EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, Operation,
        QuaternionResponse, IMU as HALIMU,
    },
    kos_proto::common::ActionResponse,
    services::OperationsServiceImpl,
};

use async_trait::async_trait;
use eyre::Result;
use imu::{HiwonderOutput, HiwonderReader, ImuFrequency, ImuReader};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tracing::{debug, error, info};

const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;
const IMU_WRITE_TIMEOUT: Duration = Duration::from_secs(4);

pub struct KBotIMU {
    _operations_service: Arc<OperationsServiceImpl>,
    imu: Arc<Mutex<HiwonderReader>>,
}

impl KBotIMU {
    pub fn new(
        operations_service: Arc<OperationsServiceImpl>,
        interface: &str,
        baud_rate: u32,
    ) -> Result<Self> {
        info!(
            "Initializing KBotIMU with interface: {} at {} baud",
            interface, baud_rate
        );

        let imu = match HiwonderReader::new(interface, baud_rate, Duration::from_millis(100), true)
        {
            Ok(imu) => {
                info!("Successfully created IMU reader");
                info!("Setting and verifying params...");
                if let Err(e) = imu.set_output_mode(
                    HiwonderOutput::QUATERNION
                        | HiwonderOutput::ANGLE
                        | HiwonderOutput::GYRO
                        | HiwonderOutput::ACC,
                    IMU_WRITE_TIMEOUT,
                ) {
                    error!("Failed to verify output mode: {}. Continuing...", e);
                } else {
                    info!("Output mode verified...");
                }
                if let Err(e) = imu.set_frequency(ImuFrequency::Hz100, IMU_WRITE_TIMEOUT) {
                    error!("Failed to verify IMU frequency: {}. Continuing...", e);
                } else {
                    info!("100Hz frequency verified...");
                }
                if let Err(e) = imu.set_bandwidth(42, IMU_WRITE_TIMEOUT) {
                    error!("Failed to verify bandwidth: {}. Continuing...", e);
                } else {
                    info!("Bandwidth verified");
                }

                info!("Reading IMU parameters...");
                let imu_parameters = match imu.read_all_registers(Duration::from_secs(1)) {
                    Ok(parameters) => parameters,
                    Err(e) => {
                        error!("Failed to read IMU parameters: {}", e);
                        return Err(eyre::eyre!("Failed to read IMU parameters: {}", e));
                    }
                };

                let hex_parameters: Vec<(String, Vec<String>)> = imu_parameters
                    .into_iter()
                    .map(|(name, values)| {
                        let name_str = format!("{:?}", name);
                        let hex_values =
                            values.into_iter().map(|v| format!("{:#04x}", v)).collect();
                        (name_str, hex_values)
                    })
                    .collect();

                if let Ok(parameters_json) = serde_json::to_string_pretty(&hex_parameters) {
                    let now = chrono::Local::now();
                    let timestamp = now.format("%Y%m%d_%H%M%S").to_string();

                    let base_log_dir: String =
                        env::var("KBOT_LOG_DIR").unwrap_or_else(|_| "/tmp/kos-kbot".to_string());
                    let log_dir = format!("{}/{}", base_log_dir, timestamp);

                    if let Err(e) = std::fs::create_dir_all(&log_dir) {
                        error!("Failed to create log directory {}: {}", log_dir, e);
                    } else {
                        let log_path = format!("{}/imu_parameters.json", log_dir);
                        match std::fs::write(&log_path, parameters_json) {
                            Ok(_) => info!("IMU parameters saved to {}", log_path),
                            Err(e) => {
                                error!("Failed to write IMU parameters to {}: {}", log_path, e)
                            }
                        }
                    }
                } else {
                    error!("Failed to serialize IMU parameters (hex) to JSON"); // Updated error message
                }

                imu
            }
            Err(e) => {
                error!("Failed to create IMU reader: {}", e);
                return Err(eyre::eyre!("Failed to create IMU reader: {}", e));
            }
        };

        Ok(KBotIMU {
            _operations_service: operations_service,
            imu: Arc::new(Mutex::new(imu)),
        })
    }
}

impl Default for KBotIMU {
    fn default() -> Self {
        unimplemented!("KBotIMU cannot be default, it requires an operations store")
    }
}

#[async_trait]
impl HALIMU for KBotIMU {
    async fn get_values(&self) -> Result<ImuValuesResponse> {
        let data = self
            .imu
            .lock()
            .map_err(|e| eyre::eyre!("Failed to lock IMU mutex: {}", e))?
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read IMU data: {}", e))?;

        debug!(
            "Reading IMU values, accel x: {}, y: {}, z: {}, gyro x: {}, y: {}, z: {}, angle x: {}, y: {}, z: {}, quaternion x: {}, y: {}, z: {}, w: {}",
            data.accelerometer.unwrap_or_default().x, data.accelerometer.unwrap_or_default().y, data.accelerometer.unwrap_or_default().z,
            data.gyroscope.unwrap_or_default().x, data.gyroscope.unwrap_or_default().y, data.gyroscope.unwrap_or_default().z,
            data.euler.unwrap_or_default().x, data.euler.unwrap_or_default().y, data.euler.unwrap_or_default().z,
            data.quaternion.unwrap_or_default().x, data.quaternion.unwrap_or_default().y, data.quaternion.unwrap_or_default().z, data.quaternion.unwrap_or_default().w,
        );

        Ok(ImuValuesResponse {
            // Accelerometer values are given in m/s^2
            accel_x: data.accelerometer.unwrap_or_default().x as f64,
            accel_y: data.accelerometer.unwrap_or_default().y as f64,
            accel_z: data.accelerometer.unwrap_or_default().z as f64,
            // Gyroscope values are given in rad/s
            gyro_x: data.gyroscope.unwrap_or_default().x as f64,
            gyro_y: data.gyroscope.unwrap_or_default().y as f64,
            gyro_z: data.gyroscope.unwrap_or_default().z as f64,
            mag_x: None,
            mag_y: None,
            mag_z: None,
            error: None,
        })
    }

    async fn get_advanced_values(&self) -> Result<ImuAdvancedValuesResponse> {
        Ok(ImuAdvancedValuesResponse {
            lin_acc_x: None,
            lin_acc_y: None,
            lin_acc_z: None,
            grav_x: None,
            grav_y: None,
            grav_z: None,
            temp: None,
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        info!("Starting IMU calibration - unimplemented");
        Ok(Operation {
            name: "operations/calibrate_imu/0".to_string(),
            metadata: None,
            done: true,
            result: None,
        })
    }

    async fn zero(
        &self,
        _duration: Option<Duration>,
        _max_retries: Option<u32>,
        _max_angular_error: Option<f32>,
        _max_vel: Option<f32>,
        _max_accel: Option<f32>,
    ) -> Result<ActionResponse> {
        // The hiwonder IMU doesn't support zeroing, so we'll return success
        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        debug!("Reading Euler angles");
        let data = self
            .imu
            .lock()
            .map_err(|e| eyre::eyre!("Failed to lock IMU mutex: {}", e))?
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read IMU data: {}", e))?;

        // Convert from radians to degrees
        Ok(EulerAnglesResponse {
            roll: data.euler.unwrap_or_default().x as f64 * RAD_TO_DEG,
            pitch: data.euler.unwrap_or_default().y as f64 * RAD_TO_DEG,
            yaw: data.euler.unwrap_or_default().z as f64 * RAD_TO_DEG,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        debug!("Reading quaternion");
        let data = self
            .imu
            .lock()
            .map_err(|e| eyre::eyre!("Failed to lock IMU mutex: {}", e))?
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read IMU data: {}", e))?;

        Ok(QuaternionResponse {
            x: data.quaternion.unwrap_or_default().x as f64,
            y: data.quaternion.unwrap_or_default().y as f64,
            z: data.quaternion.unwrap_or_default().z as f64,
            w: data.quaternion.unwrap_or_default().w as f64,
            error: None,
        })
    }
}
