import argparse

import numpy as np
import pykos
import time
import asyncio
import math
import scipy.spatial.transform as tf

kos = pykos.KOS()


imu = kos.imu

async def main(print_euler: bool, print_quat: bool, print_raw: bool, print_grav: bool, print_all: bool):
    while True:
        try:
            if print_euler or print_all:
                angles = await imu.get_euler_angles()
                print(f"roll: {round(math.degrees(angles.roll), 2):8}, pitch: {round(math.degrees(angles.pitch), 2):8}, yaw: {round(math.degrees(angles.yaw), 2):8}")
            if print_quat or print_all:
                quat = await imu.get_quaternion()
                print(f"x: {round(quat.x, 2):8}, y: {round(quat.y, 2):8}, z: {round(quat.z, 2):8}, w: {round(quat.w, 2):8}")
            if print_raw or print_all:
                raw = await imu.get_raw_data()
                print(f"x: {round(raw.x, 2):8}, y: {round(raw.y, 2):8}, z: {round(raw.z, 2):8}")
            if print_grav or print_all:
                quat = await imu.get_quaternion()
                r = tf.Rotation.from_quat([quat.x, quat.y, quat.z, quat.w], scalar_first=False)
                vec = r.apply(np.array([0, 0, -1]), inverse=True)
                # vec = r.as_euler('xyz', degrees=True)
                print(f"projected gravity vector: x: {round(vec[0], 2):8}, y: {round(vec[1], 2):8}, z: {round(vec[2], 2):8}")
        except Exception as e:
            print(e)
        time.sleep(0.1)
        print("-" * 50)


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--euler", action="store_true")
    args.add_argument("--quat", action="store_true")
    args.add_argument("--raw", action="store_true")
    args.add_argument("--grav", action="store_true")
    args.add_argument("--all", action="store_true")
    args = args.parse_args()
    asyncio.run(main(args.euler, args.quat, args.raw, args.grav, args.all))
