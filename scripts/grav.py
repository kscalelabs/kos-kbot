import time
import pykos
import numpy as np
from scipy.spatial.transform import Rotation


def get_gravity_orientation(euler_angles):
    rotation = Rotation.from_euler('xyz', euler_angles, degrees=True)
    return rotation.apply(np.array([0, 0, -1]))


def quaternion_to_euler(quaternion):
    quat_scipy = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
    rotation = Rotation.from_quat(quat_scipy)
    return rotation.as_euler('xyz', degrees=True)


def euler_to_quaternion(euler_angles):
    quat = Rotation.from_euler('xyz', euler_angles, degrees=True).as_quat()
    # return np.array([quat[3], quat[0], quat[1], quat[2]])
    return quat


def main() -> None:
    kos = pykos.KOS()
    while True:
        # quat = kos.imu.get_quaternion()
        euler_angles_msg = kos.imu.get_euler_angles()
        # quaternion = np.array([quat.w, quat.x, quat.y, quat.z])
        euler_angles = np.array([euler_angles_msg.roll, euler_angles_msg.pitch, euler_angles_msg.yaw])

        gravity_orientation = get_gravity_orientation(euler_angles)
        # inferred_euler_angles = quaternion_to_euler(quaternion)
        # inferred_quaternion = euler_to_quaternion(euler_angles)

        print(f"Gravity orientation: {gravity_orientation}")
        # print(f"Euler angles:          {euler_angles}")
        # print(f"Inferred euler angles: {inferred_euler_angles}")
        # print(f"Quaternion:          {quaternion}")
        # print(f"Inferred quaternion: {inferred_quaternion}")
        print()
        time.sleep(0.001)


if __name__ == "__main__":
    main()
