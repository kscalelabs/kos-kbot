import asyncio
import pykos

async def main():
    async with pykos.KOS() as kos:
        for id in range(60):
            try:
                await kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False, zero_position=True)
            except Exception as e:
                print(f"Failed to configure actuator {id}")

if __name__ == "__main__":
    asyncio.run(main())
