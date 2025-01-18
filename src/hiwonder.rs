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
use imu::hiwonder::HiwonderReader;
use std::sync::Arc;
use std::time::Duration;
use tracing::{debug, error, info};

pub struct KBotIMU {
    _operations_service: Arc<OperationsServiceImpl>,
    imu: Arc<HiwonderReader>,
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

        let imu = match HiwonderReader::new(interface, baud_rate) {
            Ok(imu) => {
                info!("Successfully created IMU reader");
                imu
            }
            Err(e) => {
                error!("Failed to create IMU reader: {}", e);
                return Err(eyre::eyre!("Failed to create IMU reader: {}", e));
            }
        };

        Ok(KBotIMU {
            _operations_service: operations_service,
            imu: Arc::new(imu),
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
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to get IMU data: {}", e))?;

        println!("data: {:?}", data);

        debug!(
            "Reading IMU values, accel x: {}, y: {}, z: {}, gyro x: {}, y: {}, z: {}, angle x: {}, y: {}, z: {}, quaternion x: {}, y: {}, z: {}, w: {}",
            data.accelerometer[0], data.accelerometer[1], data.accelerometer[2],
            data.gyroscope[0], data.gyroscope[1], data.gyroscope[2],
            data.angle[0], data.angle[1], data.angle[2],
            data.quaternion[0], data.quaternion[1], data.quaternion[2], data.quaternion[3],
        );

        Ok(ImuValuesResponse {
            accel_x: data.accelerometer[0] as f64,
            accel_y: data.accelerometer[1] as f64,
            accel_z: data.accelerometer[2] as f64,
            gyro_x: data.gyroscope[0] as f64,
            gyro_y: data.gyroscope[1] as f64,
            gyro_z: data.gyroscope[2] as f64,
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
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to get IMU data: {}", e))?;

        Ok(EulerAnglesResponse {
            roll: data.angle[0] as f64,
            pitch: data.angle[1] as f64,
            yaw: data.angle[2] as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        debug!("Reading quaternion");
        let data = self
            .imu
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to get IMU data: {}", e))?;

        Ok(QuaternionResponse {
            x: data.quaternion[0] as f64,
            y: data.quaternion[1] as f64,
            z: data.quaternion[2] as f64,
            w: data.quaternion[3] as f64,
            error: None,
        })
    }
}
