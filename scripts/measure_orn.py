import pykos
from scipy.spatial.transform import Rotation as R
import asyncio
import argparse

ORN_OFFSET = R.from_euler('xyz', [-1.1590576171875, -1.4337158203125, 55.6732177734375], degrees=True).inv()

async def main(offset: bool = False):
    try: 
        while True:
            kos = pykos.KOS()
            await kos.connect()
            imu_data = await kos.imu.get_euler_angles()

            if offset:
                new_euler = (ORN_OFFSET * R.from_euler('xyz', [imu_data.roll, imu_data.pitch, imu_data.yaw], degrees=True)).as_euler('xyz', degrees=True)
                imu_data.roll = new_euler[0]
                imu_data.pitch = new_euler[1]
                imu_data.yaw = new_euler[2]

            quat = R.from_euler('xyz', [imu_data.roll, imu_data.pitch, imu_data.yaw], degrees=True).as_quat()
            inverse = R.from_quat(quat).inv()
            print(f"Euler: {imu_data}")
            print(f"Quat: {quat}")
            print(f"Inverse: {inverse}")
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        print("Cancelled")
        print(f"Euler: {imu_data}")
        print(f"Quat: {quat}")
        print(f"Inverse: {inverse}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--offset", action="store_true")
    args = parser.parse_args()
    asyncio.run(main(args.offset))
