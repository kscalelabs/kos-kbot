import pykos
import asyncio

async def main():
    actuator_ids = list(range(31, 36)) + list(range(41, 46))
    async with pykos.KOS() as kos:
        # configure all actuators
        for id in actuator_ids:
            try:
                await kos.actuator.configure_actuator(actuator_id=id, kp=50, kd=5, torque_enabled=True)
            except Exception as e:
                print(f"Failed to configure actuator {id}")
        await asyncio.sleep(1)

        # get current positions
        states = (await kos.actuator.get_actuators_state(actuator_ids)).states
        current_positions = {state.actuator_id: state.position for state in states}

        commands = [{'actuator_id': id, 'position': current_positions[id], 'velocity': 0.0} for id in actuator_ids]

        await kos.actuator.command_actuators(commands)

if __name__ == "__main__":
    asyncio.run(main())
