import asyncio
import pykos

async def main():
    async with pykos.KOS() as kos:
        # for id in range(60):
        for id in [25]:
            try:
                await kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False, zero_position=True)
            except Exception as e:
                print(f"Failed to configure actuator {id}")

        # await kos.actuator.configure_actuator(actuator_id=23, torque_enabled=False, zero_position=True)
        # await kos.actuator.configure_actuator(actuator_id=25, torque_enabled=False, zero_position=True)


if __name__ == "__main__":
    asyncio.run(main())
