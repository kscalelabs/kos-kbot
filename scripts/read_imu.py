import argparse

import numpy as np
import pykos
import time
import asyncio
import math
import scipy.spatial.transform as tf

kos = pykos.KOS()


imu = kos.imu

actuator_ids = list(range(11, 16)) + list(range(21, 26)) + list(range(31, 36)) + list(range(41, 46))
times = {"euler": [], "quat": [], "raw": [], "batch": []}

async def main(print_euler: bool, print_quat: bool, print_raw: bool, print_grav: bool, print_all: bool, batch_test: bool):
    while True:
        try:
            if print_euler or print_all:
                time_start = time.perf_counter()
                angles = await imu.get_euler_angles()
                time_end = time.perf_counter()
                times["euler"].append(time_end - time_start)
                print(f"roll: {round((angles.roll), 2):8}, pitch: {round((angles.pitch), 2):8}, yaw: {round((angles.yaw), 2):8}")
            if print_quat or print_all:
                time_start = time.perf_counter()
                quat = await imu.get_quaternion()
                time_end = time.perf_counter()
                times["quat"].append(time_end - time_start)
                print(f"x: {round(quat.x, 2):8}, y: {round(quat.y, 2):8}, z: {round(quat.z, 2):8}, w: {round(quat.w, 2):8}")
            if print_raw or print_all:
                time_start = time.perf_counter()
                raw = await imu.get_imu_values()
                time_end = time.perf_counter()
                times["raw"].append(time_end - time_start)
                print(f"raw: {raw}")
                # print(f"x: {round(raw.x, 2):8}, y: {round(raw.y, 2):8}, z: {round(raw.z, 2):8}")
            if batch_test:
                time_start = time.perf_counter()
                state, quat, vals = await asyncio.gather(
                        kos.actuator.get_actuators_state(actuator_ids),
                        imu.get_quaternion(),
                        imu.get_imu_values())
                time_end = time.perf_counter()
                times["batch"].append(time_end - time_start)
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
    args.add_argument("--batch-test",action="store_true")
    args = args.parse_args()
    start_time = time.time()
    try:
        asyncio.run(main(args.euler, args.quat, args.raw, args.grav, args.all, args.batch_test))
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        if args.euler: 
            print(f"Average time taken for euler: {sum(times['euler']) / len(times['euler'])} seconds")
            print(f"Average time taken for quat: {sum(times['quat']) / len(times['quat'])} seconds")
            print(f"Average time taken for raw: {sum(times['raw']) / len(times['raw'])} seconds")
        print(f"Average for batch: {sum(times['batch']) * 1000/ len(times['batch'])} seconds")
        if args.euler:
            print(f"Min time taken for euler: {min(times['euler'])} seconds")
            print(f"Min time taken for quat: {min(times['quat'])} seconds")
            print(f"Min time taken for raw: {min(times['raw'])} seconds")
        if args.euler:
            print(f"Min time for batch: {min(times['batch']) * 1000} ms")
            print(f"Max time taken for euler: {max(times['euler'])} seconds")
            print(f"Max time taken for quat: {max(times['quat'])} seconds")
            print(f"Max time taken for raw: {max(times['raw'])} seconds")
        print(f"Max taken for batch: {max(times['batch']) * 1000} ms")
        if args.euler:
            print(f"Median time taken for euler: {sorted(times['euler'])[len(times['euler']) // 2]} seconds")
            print(f"Median time taken for quat: {sorted(times['quat'])[len(times['quat']) // 2]} seconds")
            print(f"Median time taken for raw: {sorted(times['raw'])[len(times['raw']) // 2]} seconds")
        print(f"Median time for batch: {sorted(times['batch'])[len(times['batch']) // 2] * 1000} ms")
        import numpy as np
        np_times = {key: np.array(value) for key, value in times.items()}
        np.savez('imu_read_times.npz', **np_times)
        print(f"Times saved to imu_read_times.npz")
        print(f"Total time taken: {time.time() - start_time} seconds")
