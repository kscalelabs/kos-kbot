import pykos
import asyncio

async def main():
    async with pykos.KOS() as kos:
        # configure all actuators
        for id in range(60):
            try:
                await kos.actuator.configure_actuator(actuator_id=id, kp=20, kd=1, torque_enabled=True)
            except Exception as e:
                print(f"Failed to configure actuator {id}")
        await asyncio.sleep(1)

        commands = [{'actuator_id': id, 'position': 0.0} for id in range(60)]
        await kos.actuator.command_actuators(commands)

if __name__ == "__main__":
    asyncio.run(main())
