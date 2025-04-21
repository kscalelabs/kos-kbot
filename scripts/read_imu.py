import pykos
import time
import asyncio
import math

kos = pykos.KOS()


imu = kos.imu


async def main():
    while True:
        try:
            angles = await imu.get_euler_angles()
            print(f"roll: {round(math.degrees(angles.roll), 2):8}, pitch: {round(math.degrees(angles.pitch), 2):8}, yaw: {round(math.degrees(angles.yaw), 2):8}")
        except Exception as e:
            print(e)
        time.sleep(0.1)


if __name__ == "__main__":
    asyncio.run(main())
