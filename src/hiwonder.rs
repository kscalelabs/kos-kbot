use kos::{
    hal::{
        EulerAnglesResponse, ImuAdvancedValuesResponse, ImuValuesResponse, Operation,
        QuaternionResponse, IMU as HALIMU,
    },
    kos_proto::common::{ActionResponse, Error, ErrorCode},
    services::OperationsServiceImpl,
};

use async_trait::async_trait;
use eyre::{Result, WrapErr};
use imu::hiwonder::*;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;
use tracing::{debug, error, info, trace};

pub struct KBotIMU {
    _operations_service: Arc<OperationsServiceImpl>,
    imu: Arc<Mutex<imu::hiwonder::IMU>>,
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

        let imu = match IMU::new(interface, baud_rate) {
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
        let data = self.imu.lock().unwrap()
            .read_data()
            .map_err(|e| eyre::eyre!("Failed to get IMU data: {}", e))?
            .ok_or_else(|| eyre::eyre!("No IMU data available"))?;

        info!("data: {:?}", data); 

        trace!(
            "Reading IMU values, accel x: {}, y: {}, z: {}, gyro x: {}, y: {}, z: {}, angle x: {}, y: {}, z: {}, quaternion x: {}, y: {}, z: {}, w: {}",
            data.0[0], data.0[1], data.0[2],  // acc
            data.1[0], data.1[1], data.1[2],  // gyro
            data.2[0], data.2[1], data.2[2], // angle
            data.3[0], data.3[1], data.3[2], data.3[3], // quaternion   
        );

        Ok(ImuValuesResponse {
            accel_x: data.0[0] as f64,
            accel_y: data.0[1] as f64,
            accel_z: data.0[2] as f64,
            gyro_x: data.1[0] as f64,
            gyro_y: data.1[1] as f64,
            gyro_z: data.1[2] as f64,
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
        let data = self.imu.lock().unwrap()
            .read_data()
            .map_err(|e| eyre::eyre!("Failed to get IMU data: {}", e))?
            .ok_or_else(|| eyre::eyre!("No IMU data available"))?;

        Ok(EulerAnglesResponse {
            roll: data.2[0] as f64,
            pitch: data.2[1] as f64,
            yaw: data.2[2] as f64,
            error: None,
        })
    }

    async fn get_quaternion(&self) -> Result<QuaternionResponse> {
        debug!("Reading quaternion");
        let data = self.imu.lock().unwrap()
            .read_data()
            .map_err(|e| eyre::eyre!("Failed to get IMU data: {}", e))?
            .ok_or_else(|| eyre::eyre!("No IMU data available"))?;

        Ok(QuaternionResponse {
            x: data.3[0] as f64,
            y: data.3[1] as f64,
            z: data.3[2] as f64,
            w: data.3[3] as f64,
            error: None
        })
    }
}
