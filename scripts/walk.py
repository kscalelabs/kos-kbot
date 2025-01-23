import argparse
import colorlogging
import asyncio
import time
from typing import TypedDict
import numpy as np
import pykos
import math
import logging
import onnxruntime as ort
from grav import get_gravity_orientation

logger = logging.getLogger(__name__)

r04_ids = [31, 34, 41, 44]
r03_ids = [11, 12, 21, 22, 32, 33, 42, 43]
r02_ids = [13, 14, 15, 23, 24, 25, 35, 45]

kos_to_urdf_offsets = {
    'left_shoulder_pitch_03': 5.306456089019775,
    'left_shoulder_roll_03': -1.680882215499878,
    'left_shoulder_yaw_02': -13.326627731323242,
    'left_elbow_02': -0.3680419921875,
    'left_wrist_02': -161.1829071044922,
    'right_shoulder_pitch_03': -1.593018651008606,
    'right_shoulder_roll_03': -0.07693525403738022,
    'right_shoulder_yaw_02': -45.53900146484375,
    'right_elbow_02': -1.087639331817627,
    'right_wrist_02': -138.46835327148438,
    'left_hip_pitch_04': 7.679536819458008,
    'left_hip_roll_03': -0.8239940404891968,
    'left_hip_yaw_03': -21.67634391784668,
    'left_knee_04': 19.759246826171875,
    'left_ankle_02': -25.236019134521484,
    'right_hip_pitch_04': -5.85582160949707,
    'right_hip_roll_03': -0.9338234663009644,
    'right_hip_yaw_03': -14.05172061920166,
    'right_knee_04': -12.865180969238281,
    'right_ankle_02': 13.58477783203125
}

joint_name_to_id = {
    # Left arm
    'left_shoulder_pitch_03': 11,
    'left_shoulder_roll_03': 12,
    'left_shoulder_yaw_02': 13,
    'left_elbow_02': 14,
    'left_wrist_02': 15,
    # Right arm
    'right_shoulder_pitch_03': 21,
    'right_shoulder_roll_03': 22,
    'right_shoulder_yaw_02': 23,
    'right_elbow_02': 24,
    'right_wrist_02': 25,
    # Left leg
    'left_hip_pitch_04': 31,
    'left_hip_roll_03': 32,
    'left_hip_yaw_03': 33,
    'left_knee_04': 34,
    'left_ankle_02': 35,
    # Right leg
    'right_hip_pitch_04': 41,
    'right_hip_roll_03': 42,
    'right_hip_yaw_03': 43,
    'right_knee_04': 44,
    'right_ankle_02': 45
}

joint_name_list = [
    'left_hip_pitch_04',
    'left_shoulder_pitch_03', 
    'right_hip_pitch_04',
    'right_shoulder_pitch_03',
    'left_hip_roll_03',
    'left_shoulder_roll_03',
    'right_hip_roll_03', 
    'right_shoulder_roll_03',
    'left_hip_yaw_03',
    'left_shoulder_yaw_02',
    'right_hip_yaw_03',
    'right_shoulder_yaw_02',
    'left_knee_04',
    'left_elbow_02',
    'right_knee_04',
    'right_elbow_02',
    'left_ankle_02',
    'left_wrist_02',
    'right_ankle_02',
    'right_wrist_02'
]

arm_joint_name_list = [
    'left_shoulder_pitch_03',
    'left_shoulder_roll_03',
    'left_shoulder_yaw_02',
    'left_elbow_02',
    'left_wrist_02',
    'right_shoulder_pitch_03',
    'right_shoulder_roll_03',
    'right_shoulder_yaw_02',
    'right_elbow_02',
    'right_wrist_02'
]

class Observation(TypedDict):
    angular_velocity: np.ndarray
    vel_commands: np.ndarray
    gravity_orientation: np.ndarray
    joint_angles: np.ndarray
    joint_velocities: np.ndarray
    past_action: np.ndarray

class URDFToKOSConverter:
    """Converts URDF joint angles to KOS joint angles and vice versa."""
    
    def __init__(self) -> None:
        self.kos_to_urdf_offsets = kos_to_urdf_offsets
        self.joint_name_to_id = joint_name_to_id
        self.signs = {
            # Left arm
            "left_shoulder_pitch_03": 1,
            "left_shoulder_roll_03": -1,
            "left_shoulder_yaw_02": 1,
            "left_elbow_02": 1,
            "left_wrist_02": 1,
            # Right arm
            "right_shoulder_pitch_03": 1,
            "right_shoulder_roll_03": 1,
            "right_shoulder_yaw_02": -1,
            "right_elbow_02": 1,
            "right_wrist_02": 1,
            # Left leg
            "left_hip_pitch_04": 1,
            "left_hip_roll_03": 1,
            "left_hip_yaw_03": 1,
            "left_knee_04": -1,
            "left_ankle_02": 1,
            # Right leg
            "right_hip_pitch_04": 1,
            "right_hip_roll_03": 1,
            "right_hip_yaw_03": 1,
            "right_knee_04": -1,
            "right_ankle_02": -1
        }

        # # TODO: IDK
        # self.signs = {k: -v for k, v in self.signs.items()}

    def kos_to_urdf(self, joint_name: str, kos_value: float, radians: bool = True) -> float:
        """Convert a KOS joint angle to URDF joint angle.
        
        Args:
            joint_name: Name of the joint
            kos_value: Joint angle in KOS frame (degrees)
            radians: If True, return angle in radians, else in degrees
            
        Returns:
            URDF joint angle in radians or degrees
        """
        # Apply offset and sign correction
        urdf_value = (kos_value + self.kos_to_urdf_offsets[joint_name]) * self.signs[joint_name]
        
        # Convert to radians if requested
        if radians:
            urdf_value = math.radians(urdf_value)
            
        return urdf_value

    def urdf_to_kos(self, joint_name: str, urdf_value: float, radians: bool = True) -> float:
        """Convert a URDF joint angle to KOS joint angle.
        
        Args:
            joint_name: Name of the joint
            urdf_value: Joint angle in URDF frame (radians or degrees)
            radians: If True, input angle is in radians, else in degrees
            
        Returns:
            KOS joint angle in degrees
        """
        # Convert from radians if needed
        if radians:
            urdf_value = math.degrees(urdf_value)
            
        # Remove sign correction and offset
        kos_value = (urdf_value * self.signs[joint_name]) - self.kos_to_urdf_offsets[joint_name]
        
        return kos_value
    
    def kos_velocity_to_urdf_velocity(self, joint_name: str, kos_velocity: float, radians: bool = True) -> float:
        urdf_velocity = kos_velocity * self.signs[joint_name]
        if radians:
            return math.radians(urdf_velocity)
        else:
            return urdf_velocity
    
async def get_joint_data(kos: pykos.KOS, urdfconverter: URDFToKOSConverter) -> tuple[list[float], list[float]]:
    # TODO: Check that this ordering is correct
    ids = [joint_name_to_id[name] for name in joint_name_list]
    response = await kos.actuator.get_actuators_state(ids)
    states = response.states
    state_dict = {state.actuator_id: state for state in states}

    angles = [state_dict[id].position for id in ids]
    velocities = [state_dict[id].velocity for id in ids]

    urdf_angles = [urdfconverter.kos_to_urdf(joint_name, angle) for joint_name, angle in zip(joint_name_list, angles)]
    urdf_velocities = [urdfconverter.kos_velocity_to_urdf_velocity(joint_name, velocity) for joint_name, velocity in zip(joint_name_list, velocities)]
    return urdf_angles, urdf_velocities

async def get_observation(kos: pykos.KOS, urdfconverter: URDFToKOSConverter) -> Observation:
    euler_angles, data, (angles, velocities) = await asyncio.gather(
        kos.imu.get_euler_angles(),
        kos.imu.get_imu_values(),
        get_joint_data(kos, urdfconverter),
    )

    gyro_x = data.gyro_x or 0
    gyro_y = data.gyro_y or 0
    gyro_z = data.gyro_z or 0

    gravity_orientation = get_gravity_orientation(np.array([euler_angles.roll, euler_angles.pitch, euler_angles.yaw]))
    normalized_gravity_orientation = gravity_orientation / np.linalg.norm(gravity_orientation)

    return Observation(
        gravity_orientation=normalized_gravity_orientation,
        joint_angles=np.array(angles),
        joint_velocities=np.array(velocities),
        vel_commands=np.zeros(3),
        past_action=np.zeros(len(joint_name_list)),
        angular_velocity=np.array([gyro_x, gyro_y, gyro_z])
    )

async def send_commands(kos: pykos.KOS, urdfconverter: URDFToKOSConverter, commands: dict[str, float]) -> None:
    kos_commands = {
        joint_name_to_id[joint_name]: urdfconverter.urdf_to_kos(joint_name, command) for joint_name, command in commands.items()
    }
    commands = [
        {
            "actuator_id": id,
            "position": position
        }
        for id, position in kos_commands.items()
    ]
    # await kos.actuator.command_actuators(commands)

async def run_policy(policy: str) -> None:
    policy = ort.InferenceSession(policy)

    async with pykos.KOS() as kos:
        enable_torque = False

        # Configure actuators
        for id in r04_ids:
            # await kos.actuator.configure_actuator(actuator_id = id, kp=300, kd=5, max_torque=120, torque_enabled=enable_torque)
            await kos.actuator.configure_actuator(actuator_id = id, kp=150, kd=5, max_torque=120, torque_enabled=enable_torque)


        for id in r03_ids:
            await kos.actuator.configure_actuator(actuator_id = id, kp=150, kd=5, max_torque=60, torque_enabled=enable_torque)

        for id in r02_ids:
            await kos.actuator.configure_actuator(actuator_id = id, kp=40, kd=5, max_torque=17, torque_enabled=enable_torque)

        urdfconverter = URDFToKOSConverter()

        vel_commands = {
            "x_lin_vel": 0.5,
            "y_lin_vel": 0.0,
            "z_ang_vel": 0.0
        }

        policy_freq = 1  # hz
        policy_dt = 1 / policy_freq
        actions = np.zeros(len(joint_name_list))

        max_command = 0

        scale_factor = 0.02 # Adjust this value to control scaling

        num_commands = 0
        last_time = time.time()

        joint_commands = {s: 0.0 for s in joint_name_list}

        while True:
            process_start = time.time()

            obs, _ = await asyncio.gather(
                get_observation(kos, urdfconverter),
                send_commands(kos, urdfconverter, joint_commands),
            )

            obs["vel_commands"] = np.array([vel_commands["x_lin_vel"], vel_commands["y_lin_vel"], vel_commands["z_ang_vel"]])
            obs["past_action"] = actions

            # Concatenate all observation components into a single array
            input_tensor = np.concatenate([
                obs["angular_velocity"],  # (3)
                obs["vel_commands"],  # (3)
                obs["gravity_orientation"],  #(3)
                obs["joint_angles"] / scale_factor,  # (20)
                obs["joint_velocities"],  # (20)
                obs["past_action"],  # (20)
            ]).astype(np.float32)

            # Add batch dimension
            input_tensor = input_tensor[np.newaxis, :]

            # Run inference
            actions = policy.run(None, {"obs": input_tensor})[0][0]

            joint_commands = {
                joint_name_list[i]: actions[i] for i in range(len(actions))
            }

            # Scale down joint commands
            for joint_name in joint_commands:
                joint_commands[joint_name] *= scale_factor

            # Turn off arm joints.
            for arm_joint_name in arm_joint_name_list:
                joint_commands[arm_joint_name] = 0.0

            # Clip joint commands to +/- pi/6 radians
            for joint_name in joint_commands:
                joint_commands[joint_name] = np.clip(joint_commands[joint_name], -math.pi/6, math.pi/6)

            # Track max joint command value across all iterations
            max_command = max(max_command, max(abs(val) for val in joint_commands.values()))
            avg_command = np.mean([abs(val) for val in joint_commands.values()])
            print(f"Joint commands - Max: {max_command:.4f} rad ({math.degrees(max_command):.2f}°), Avg: {avg_command:.4f} rad ({math.degrees(avg_command):.2f}°)")

            sleep_time = max(0, process_start + policy_dt - time.time())

            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                logger.warning("Policy is running slower than expected")

            num_commands += 1
            if (cur_time := time.time()) > last_time + 1.0:
                elapsed_time, last_time = cur_time - last_time, cur_time
                logger.info(f"Commands per second: {num_commands / elapsed_time:.2f}")
                num_commands = 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=str, required=True)
    args = parser.parse_args()

    colorlogging.configure()

    asyncio.run(run_policy(args.policy))


if __name__ == "__main__":
    main()
