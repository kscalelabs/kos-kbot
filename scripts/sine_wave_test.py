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

# List of joints to test simultaneously
JOINTS_TO_TEST = [
    'left_knee_04',
    'right_knee_04',
    'left_hip_pitch_04',
    'right_hip_pitch_04'
]

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
    await kos.actuator.command_actuators(commands)

async def run_sine_wave_test(amplitude: float, frequency: float, duration: float) -> None:
    """Run a sine wave test on multiple joints simultaneously.
    
    Args:
        amplitude: Amplitude of the sine wave in radians
        frequency: Frequency of the sine wave in Hz
        duration: Duration of the test in seconds
    """
    # Validate all joints
    for joint_name in JOINTS_TO_TEST:
        if joint_name not in joint_name_to_id:
            raise ValueError(f"Invalid joint name: {joint_name}")

    async with pykos.KOS() as kos:
        enable_torque = True

        # Configure all joints
        for joint_name in JOINTS_TO_TEST:
            joint_id = joint_name_to_id[joint_name]
            if joint_id in r04_ids:
                await kos.actuator.configure_actuator(actuator_id=joint_id, kp=250, kd=5, max_torque=120, torque_enabled=enable_torque)
            elif joint_id in r03_ids:
                await kos.actuator.configure_actuator(actuator_id=joint_id, kp=150, kd=5, max_torque=60, torque_enabled=enable_torque)
            else:  # r02_ids
                await kos.actuator.configure_actuator(actuator_id=joint_id, kp=40, kd=5, max_torque=17, torque_enabled=enable_torque)

        urdfconverter = URDFToKOSConverter()
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Generate sine wave commands for all joints
            joint_commands = {}
            for joint_name in JOINTS_TO_TEST:
                command = amplitude * math.sin(2 * math.pi * frequency * t)
                joint_commands[joint_name] = command
            
            # Send commands to all joints simultaneously
            await send_commands(kos, urdfconverter, joint_commands)
            
            # Get current joint states for logging
            angles, velocities = await get_joint_data(kos, urdfconverter)
            
            # Print status for all joints
            print(f"\nTime: {t:.2f}s")
            for joint_name in JOINTS_TO_TEST:
                joint_idx = joint_name_list.index(joint_name)
                current_angle = angles[joint_idx]
                command = joint_commands[joint_name]
                print(f"{joint_name}: Command: {math.degrees(command):.2f}°, Current: {math.degrees(current_angle):.2f}°")
            
            # Sleep to maintain reasonable control frequency
            await asyncio.sleep(0.01)  # 100Hz control rate


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--amplitude", type=float, default=30.0, help="Amplitude of sine wave in degrees (default: 30)")
    parser.add_argument("--frequency", type=float, default=1, help="Frequency of sine wave in Hz (default: 0.2, i.e., 5 seconds per oscillation)")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration of test in seconds (default: 10)")
    args = parser.parse_args()

    colorlogging.configure()

    # Convert amplitude from degrees to radians
    amplitude_rad = math.radians(args.amplitude)

    asyncio.run(run_sine_wave_test(amplitude_rad, args.frequency, args.duration))


if __name__ == "__main__":
    main()
