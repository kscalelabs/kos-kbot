import pykos
import asyncio

id = 32

async def main():
    async with pykos.KOS() as kos:
        await kos.actuator.configure_actuator(actuator_id=id, kp=10, kd=1, torque_enabled=True)

        await asyncio.sleep(1)

        await kos.actuator.command_actuators([{'actuator_id': id, 'position': -20}])

if __name__ == "__main__":
    asyncio.run(main())
