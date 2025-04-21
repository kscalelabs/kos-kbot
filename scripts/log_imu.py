import pykos
import asyncio
import scipy.spatial.transform as tf
import numpy as np

async def main():
    csv_out = open("imu.csv", "w")
    csv_out.write("roll, pitch, yaw, x, y, z, w, gx, gy, gz, gyro_x, gyro_y, gyro_z\n")
    async with pykos.KOS() as kos:
        try:
            while True:
                imu_data = await kos.imu.get_imu_values()
                euler = await kos.imu.get_euler_angles()
                quat = await kos.imu.get_quaternion()

                r = tf.Rotation.from_quat([quat.x, quat.y, quat.z, quat.w], scalar_first=True)
                vec = r.apply(np.array([0, 0, -1]), inverse=True)
                csv_out.write(f"{euler.roll}, {euler.pitch}, {euler.yaw}, {quat.x}, {quat.y}, {quat.z}, {quat.w}, {vec[0]}, {vec[1]}, {vec[2]}, {imu_data.gyro_x}, {imu_data.gyro_y}, {imu_data.gyro_z}\n")
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            csv_out.close()
            print("Cancelled")

if __name__ == "__main__":
    asyncio.run(main())
