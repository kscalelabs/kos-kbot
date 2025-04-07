import pykos
import asyncio

async def main():
    async with pykos.KOS() as kos:
        for id in list(range(21, 26)):
            try:
                await kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)
            except Exception as e:
                print(f"Failed to disable actuator {id}")

if __name__ == "__main__":
    asyncio.run(main())