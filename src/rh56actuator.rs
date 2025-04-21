use crate::{Arc, Hand, Operation, OperationsServiceImpl};
use async_trait::async_trait;
use eyre::Result;
use kos::{
    hal::{ActionResponse, Actuator, ActuatorCommand, CalibrateActuatorRequest},
    kos_proto::{
        actuator::*,
        common::Error as KosError,
        common::{ActionResult, ErrorCode},
    },
};

pub struct RH56Actuator {
    hand: Arc<Hand>,
    id_offset: u8,
}

impl RH56Actuator {
    pub async fn new(
        _operations_service: Arc<OperationsServiceImpl>,
        serial_port: &str,
        hand_id: u8,
        id_offset: u8,
    ) -> Result<Self> {
        let hand = Hand::new(serial_port, 115200, hand_id).await?;
        hand.clone().start().await?;

        Ok(RH56Actuator { hand, id_offset })
    }
}

#[async_trait]
impl Actuator for RH56Actuator {
    async fn command_actuators(&self, commands: Vec<ActuatorCommand>) -> Result<Vec<ActionResult>> {
        let mut results = vec![];
        for command in commands {
            let motor_id = command.actuator_id as u8;

            if (self.id_offset..=self.id_offset + 5).contains(&motor_id) {
                if let Some(position) = command.position {
                    let hand_position = (position * 10.0) as i32;
                    let hand_position = hand_position.clamp(0, 1000);
                    let finger_idx = (motor_id - self.id_offset) as usize;

                    let result = self
                        .hand
                        .set_finger_position(finger_idx, hand_position)
                        .await;

                    let success = result.is_ok();
                    let error = result.err().map(|e| KosError {
                        code: ErrorCode::HardwareFailure as i32,
                        message: e.to_string(),
                    });

                    results.push(ActionResult {
                        actuator_id: command.actuator_id,
                        success,
                        error,
                    });
                }
            }
        }
        Ok(results)
    }

    async fn configure_actuator(
        &self,
        _config: ConfigureActuatorRequest,
    ) -> Result<ActionResponse> {
        Ok(ActionResponse {
            success: true,
            error: None,
        })
    }

    async fn calibrate_actuator(&self, _request: CalibrateActuatorRequest) -> Result<Operation> {
        Ok(Operation::default())
    }

    async fn get_actuators_state(
        &self,
        actuator_ids: Vec<u32>,
    ) -> Result<Vec<ActuatorStateResponse>> {
        let mut responses = vec![];
        for id in actuator_ids {
            if (self.id_offset..=self.id_offset + 5).contains(&(id as u8)) {
                let finger_idx = (id as u8 - self.id_offset) as usize;

                let position = match self.hand.get_finger_position(finger_idx).await {
                    Ok(pos) => Some(pos as f64 / 10.0),
                    Err(_) => None,
                };

                responses.push(ActuatorStateResponse {
                    actuator_id: id,
                    online: true,
                    position,
                    velocity: Some(0.0),
                    torque: None,
                    temperature: None,
                    voltage: None,
                    current: None,
                    faults: Vec::new(),
                });
            }
        }
        Ok(responses)
    }
}