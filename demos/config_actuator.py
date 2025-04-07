import pykos
import asyncio

async def main():
    async with pykos.KOS() as kos:
        # configure all actuators
        for id in [31, 32, 33, 34, 35, 41, 42, 43, 44, 45]:
            try:
                if id == 31 or id == 41:
                    kp = 100
                else:
                    kp = 50
                await kos.actuator.configure_actuator(actuator_id=id, kp=kp, kd=5, torque_enabled=True)
                position = (await kos.actuator.get_actuators_state([id])).states[0].position
                commands = [{'actuator_id': id, 'position': position}]
                await kos.actuator.command_actuators(commands)
            except Exception as e:
                print(f"Failed to configure actuator {id}, error: {e}")
        
        # await asyncio.sleep(1)
        # commands = [{'actuator_id': id, 'position': position, 'velocity': 0.0} for id in range(30, 60)]
        # await kos.actuator.command_actuators(commands)

if __name__ == "__main__":

    asyncio.run(main())
