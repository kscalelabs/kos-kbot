use crate::{Arc, Operation, OperationsServiceImpl};
use async_trait::async_trait;
use eyre::Result;
use kos::{
    hal::{ActionResponse, Actuator, ActuatorCommand, CalibrateActuatorRequest},
    kos_proto::{actuator::*, common::ActionResult},
};

struct ActuatorWithRange {
    actuator: Box<dyn Actuator + Send + Sync>,
    id_range: std::ops::RangeInclusive<u8>,
}

pub struct ProxyActuator {
    actuators: Vec<ActuatorWithRange>,
}

impl ProxyActuator {
    pub fn new(
        actuators: Vec<(
            Box<dyn Actuator + Send + Sync>,
            std::ops::RangeInclusive<u8>,
        )>,
    ) -> Self {
        let actuators = actuators
            .into_iter()
            .map(|(actuator, range)| ActuatorWithRange {
                actuator,
                id_range: range,
            })
            .collect();

        ProxyActuator { actuators }
    }

    fn find_actuator_for_id(&self, id: u32) -> Option<&(dyn Actuator + Send + Sync)> {
        self.actuators
            .iter()
            .find(|a| a.id_range.contains(&(id as u8)))
            .map(|a| a.actuator.as_ref())
    }
}

#[async_trait]
impl Actuator for ProxyActuator {
    async fn command_actuators(&self, commands: Vec<ActuatorCommand>) -> Result<Vec<ActionResult>> {
        let mut all_results = Vec::new();

        // Group commands by actuator
        for command in commands {
            if let Some(actuator) = self.find_actuator_for_id(command.actuator_id) {
                let result = actuator.command_actuators(vec![command]).await?;
                all_results.extend(result);
            }
        }

        Ok(all_results)
    }

    async fn configure_actuator(&self, config: ConfigureActuatorRequest) -> Result<ActionResponse> {
        if let Some(actuator) = self.find_actuator_for_id(config.actuator_id) {
            actuator.configure_actuator(config).await
        } else {
            Ok(ActionResponse {
                success: false,
                error: None,
            })
        }
    }

    async fn calibrate_actuator(&self, request: CalibrateActuatorRequest) -> Result<Operation> {
        if let Some(actuator) = self.find_actuator_for_id(request.actuator_id) {
            actuator.calibrate_actuator(request).await
        } else {
            Ok(Operation::default())
        }
    }

    async fn get_actuators_state(
        &self,
        actuator_ids: Vec<u32>,
    ) -> Result<Vec<ActuatorStateResponse>> {
        let mut all_responses = Vec::new();

        for id in actuator_ids {
            if let Some(actuator) = self.find_actuator_for_id(id) {
                let responses = actuator.get_actuators_state(vec![id]).await?;
                all_responses.extend(responses);
            }
        }

        Ok(all_responses)
    }
}