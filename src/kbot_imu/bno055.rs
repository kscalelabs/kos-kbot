use kos::{
    hal::{
        EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, Operation,
        QuaternionResponse, IMU as HALIMU,
    },
    kos_proto::common::{ActionResponse, Error as ProtoError, ErrorCode},
    services::OperationsServiceImpl,
};

use async_trait::async_trait;
use eyre::Result;
use imu::bno055::*;
use std::sync::Arc;
use std::time::Duration;
use tracing::{debug, error, info, warn};

pub struct KBotIMU {
    _operations_service: Arc<OperationsServiceImpl>,
    imu: Arc<Bno055Reader>,
}

impl KBotIMU {
    pub fn new(
        operations_service: Arc<OperationsServiceImpl>,
        i2c_bus: &str,
    ) -> Result<Self> {
        info!(
            "Initializing KBotIMU (BNO055) with interface: {}",
            i2c_bus
        );

        let imu_reader = match Bno055Reader::new(i2c_bus) {
            Ok(imu) => {
                info!("Successfully created BNO055 IMU reader");
                imu
            }
            Err(e) => {
                error!("Failed to create BNO055 IMU reader: {}", e);
                return Err(eyre::eyre!("Failed to create BNO055 IMU reader: {}", e));
            }
        };

        Ok(KBotIMU {
            _operations_service: operations_service,
            imu: Arc::new(imu_reader),
        })
    }
}

impl Default for KBotIMU {
    fn default() -> Self {
        unimplemented!("KBotIMU cannot be default, it requires an operations store and I2C bus")
    }
}

#[async_trait]
impl HALIMU for KBotIMU {
    async fn get_values(&self) -> Result<ImuValuesResponse> {
        let data = self
            .imu
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read BNO055 data: {}", e))?;

        debug!(
            "Reading BNO055 values, accel: {:?}, gyro: {:?}, mag: {:?}",
            data.accelerometer, data.gyroscope, data.magnetometer
        );

        Ok(ImuValuesResponse {
            // Accelerometer values are in m/s^2
            accel_x: data.accelerometer.x as f64,
            accel_y: data.accelerometer.y as f64,
            accel_z: data.accelerometer.z as f64,
            // Gyroscope values are in deg/s
            gyro_x: data.gyroscope.x as f64,
            gyro_y: data.gyroscope.y as f64,
            gyro_z: data.gyroscope.z as f64,
            // Magnetometer values are in microTesla (µT)
            mag_x: Some(data.magnetometer.x as f64),
            mag_y: Some(data.magnetometer.y as f64),
            mag_z: Some(data.magnetometer.z as f64),
            error: None,
        })
    }

    async fn get_advanced_values(&self) -> Result<ImuAdvancedValuesResponse> {
        let data = self
            .imu
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read BNO055 data: {}", e))?;

        debug!(
            "Reading BNO055 advanced values, lin_accel: {:?}, gravity: {:?}, temp: {}",
            data.linear_acceleration, data.gravity, data.temperature
        );

        Ok(ImuAdvancedValuesResponse {
            // Linear acceleration in m/s^2
            lin_acc_x: Some(data.linear_acceleration.x as f64),
            lin_acc_y: Some(data.linear_acceleration.y as f64),
            lin_acc_z: Some(data.linear_acceleration.z as f64),
            // Gravity vector in m/s^2
            grav_x: Some(data.gravity.x as f64),
            grav_y: Some(data.gravity.y as f64),
            grav_z: Some(data.gravity.z as f64),
            // Temperature in Celsius
            temp: Some(data.temperature as f32),
            error: None,
        })
    }

    async fn calibrate(&self) -> Result<Operation> {
        info!("IMU calibration requested - BNO055 calibrates automatically in NDOF mode.");
        Ok(Operation {
            name: "operations/calibrate_imu/bno055".to_string(),
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
        info!("IMU zero requested - BNO055 handles this via continuous calibration.");
        Ok(ActionResponse {
            success: false,
            error: Some(ProtoError {
                code: ErrorCode::HardwareFailure as i32,
                message: "BNO055 does not support zeroing".to_string(),
            }),
        })
    }

    async fn get_euler(&self) -> Result<EulerAnglesResponse> {
        let data = self
            .imu
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read BNO055 data: {}", e))?;

        debug!("Reading BNO055 Euler angles: {:?}", data.euler);

        // Euler angles are in degrees
        Ok(EulerAnglesResponse {
            roll: data.euler.roll as f64, // x-axis
            pitch: data.euler.pitch as f64, // y-axis
            yaw: data.euler.yaw as f64,   // z-axis
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        let data = self
            .imu
            .get_data()
            .map_err(|e| eyre::eyre!("Failed to read BNO055 data: {}", e))?;

        debug!("Reading BNO055 quaternion: {:?}", data.quaternion);

        Ok(QuaternionResponse {
            w: data.quaternion.w as f64,
            x: data.quaternion.x as f64,
            y: data.quaternion.y as f64,
            z: data.quaternion.z as f64,
            error: None,
        })
    }
}
