import pykos
import asyncio

async def main():
    async with pykos.KOS() as kos:
        print(await kos.actuator.get_actuators_state([31, 32, 33, 34, 35, 41, 42, 43, 44, 45]))

if __name__ == "__main__":
    asyncio.run(main())
