"""Real robot walking deployment script using PyKOS interface."""

import argparse
import asyncio
import math
import time
from copy import deepcopy
from typing import Dict, List, Tuple, Union

import numpy as np
import pygame
from kinfer.inference.python import ONNXModel
import pykos
from pykos import KOS
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

# Ordered joint names for policy (legs only, top-down, left-right order)
JOINT_NAME_LIST = [
    "L_hip_y",    # Left leg, top to bottom
    "L_hip_x",
    "L_hip_z", 
    "L_knee",
    "L_ankle",
    "R_hip_y",    # Right leg, top to bottom
    "R_hip_x",
    "R_hip_z",
    "R_knee",
    "R_ankle"
]

# Joint mapping for KOS
JOINT_NAME_TO_ID = {
    # Left leg
    "L_hip_y": 31,
    "L_hip_x": 32,
    "L_hip_z": 33,
    "L_knee": 34,
    "L_ankle": 35,
    # Right leg
    "R_hip_y": 41,
    "R_hip_x": 42,
    "R_hip_z": 43,
    "R_knee": 44,
    "R_ankle": 45
}

# Joint signs for correct motion direction
JOINT_SIGNS = {
    # Left leg
    "L_hip_y": 1,
    "L_hip_x": 1,
    "L_hip_z": 1,
    "L_knee": -1,
    "L_ankle": 1,
    # Right leg
    "R_hip_y": 1,
    "R_hip_x": 1,
    "R_hip_z": 1,
    "R_knee": 1,
    "R_ankle": -1
}

def handle_keyboard_input() -> None:
    """Handle keyboard input for velocity commands."""
    global x_vel_cmd, y_vel_cmd, yaw_vel_cmd
    
    keys = pygame.key.get_pressed()

    if keys[pygame.K_UP]:
        x_vel_cmd += 0.0005
    if keys[pygame.K_DOWN]:
        x_vel_cmd -= 0.0005
    if keys[pygame.K_LEFT]:
        y_vel_cmd += 0.0005
    if keys[pygame.K_RIGHT]:
        y_vel_cmd -= 0.0005
    if keys[pygame.K_a]:
        yaw_vel_cmd += 0.001
    if keys[pygame.K_z]:
        yaw_vel_cmd -= 0.001

class RobotState:
    """Tracks robot state and handles offsets."""
    def __init__(self, joint_names: List[str], joint_signs: Dict[str, float]):
        self.joint_offsets = {name: 0.0 for name in joint_names}
        self.joint_signs = joint_signs
        self.orn_offset = None

    async def offset_in_place(self, kos: KOS, joint_names: List[str]) -> None:
        """Capture current position as zero offset."""
        # Get current joint positions
        states = await kos.actuator.get_actuators_state([JOINT_NAME_TO_ID[name] for name in joint_names])
        current_positions = {name: states.states[i].position for i, name in enumerate(joint_names)}
        
        # Store negative of current positions as offsets
        self.joint_offsets = {name: -pos for name, pos in current_positions.items()}

        # Store IMU offset
        imu_data = await kos.imu.get_euler_angles()
        initial_quat = R.from_euler('xyz', [imu_data.roll, imu_data.pitch, imu_data.yaw], degrees=True).as_quat()
        self.orn_offset = R.from_quat(initial_quat).inv()

    async def get_obs(self, kos: KOS) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get robot state with offset compensation."""
        # Get joint states
        states = await kos.actuator.get_actuators_state([JOINT_NAME_TO_ID[name] for name in JOINT_NAME_LIST])
        
        # Apply offsets and signs to positions
        q = np.array([
            (states.states[i].position + self.joint_offsets[name]) * self.joint_signs[name]
            for i, name in enumerate(JOINT_NAME_LIST)
        ], dtype=np.float32)
        
        # Apply signs to velocities
        dq = np.array([
            states.states[i].velocity * self.joint_signs[name]
            for i, name in enumerate(JOINT_NAME_LIST)
        ], dtype=np.float32)

        # Get IMU data with offset compensation
        imu_data = await kos.imu.get_euler_angles()
        current_quat = R.from_euler('xyz', [imu_data.roll, imu_data.pitch, imu_data.yaw], degrees=True).as_quat()
        if self.orn_offset is not None:
            # Apply the offset by quaternion multiplication
            current_rot = R.from_quat(current_quat)
            quat = (self.orn_offset * current_rot).as_quat()
        else:
            quat = current_quat

        # Calculate gravity vector with offset compensation
        r = R.from_quat(quat)
        gvec = r.apply(np.array([0.0, 0.0, -1.0]), inverse=True)

        return q, dq, quat, gvec

    def apply_command(self, position: float, joint_name: str) -> float:
        """Apply offset and sign to outgoing command."""
        return (position - self.joint_offsets[joint_name]) * self.joint_signs[joint_name]

async def pd_control(
    kos: KOS,
    target_q: np.ndarray,
    q: np.ndarray,
    kp: np.ndarray,
    dq: np.ndarray,
    kd: np.ndarray,
    default: np.ndarray,
) -> None:
    """Apply PD control to the robot."""
    tau = kp * (target_q + default - q) - kd * dq
    
    # Send position commands to each joint
    for i, joint_name in enumerate(JOINT_NAME_LIST):
        joint_id = JOINT_NAME_TO_ID[joint_name]
        position = float(target_q[i] + default[i]) * JOINT_SIGNS[joint_name]
        await kos.actuator.command_actuators([{"actuator_id": joint_id, "position": position}])

async def run_robot(
    kos: KOS,
    policy: ONNXModel,
    model_info: Dict[str, Union[float, List[float], str]],
    keyboard_use: bool = False,
    duration: float = 60.0,
) -> None:
    """Run the walking policy on the real robot."""
    
    # Initialize robot state handler
    robot_state = RobotState(JOINT_NAME_LIST, JOINT_SIGNS)
    
    # Configure motors
    print("Configuring motors...")
    leg_ids = [JOINT_NAME_TO_ID[name] for name in JOINT_NAME_LIST]
    
    # First disable torque and zero positions
    for joint_id in leg_ids:
        await kos.actuator.configure_actuator(actuator_id=joint_id, torque_enabled=False, zero_position=True)
    await asyncio.sleep(1)

    # Capture current position as zero
    print("Capturing current position as zero...")
    await robot_state.offset_in_place(kos, JOINT_NAME_LIST)

    tau_limit = np.array(list(model_info["robot_effort"]) + list(model_info["robot_effort"])) * model_info["tau_factor"]
    kps = np.array(list(model_info["robot_stiffness"]) + list(model_info["robot_stiffness"]))
    kds = np.array(list(model_info["robot_damping"]) + list(model_info["robot_damping"]))

    # Configure gains for each joint
    for i, joint_name in enumerate(JOINT_NAME_LIST):
        joint_id = JOINT_NAME_TO_ID[joint_name]
        await kos.actuator.configure_actuator(
            actuator_id=joint_id,
            kp=float(kps[i]),
            kd=float(kds[i]),
            max_torque=float(tau_limit[i]),
            torque_enabled=True
        )

    # Initialize policy state
    default = np.zeros(model_info["num_actions"])
    target_q = np.zeros(model_info["num_actions"], dtype=np.float32)
    prev_actions = np.zeros(model_info["num_actions"], dtype=np.float32)
    hist_obs = np.zeros(model_info["num_observations"], dtype=np.float32)
    count_policy = 0

    try:
        while True:
            process_start = time.time()
            if keyboard_use:
                handle_keyboard_input()

            # Get robot state with offset compensation
            q, dq, quat, gvec = await robot_state.get_obs(kos)

            # Prepare policy inputs
            input_data = {
                "x_vel.1": np.array([x_vel_cmd], dtype=np.float32),
                "y_vel.1": np.array([y_vel_cmd], dtype=np.float32),
                "rot.1": np.array([yaw_vel_cmd], dtype=np.float32),
                "t.1": np.array([count_policy * model_info["policy_dt"]], dtype=np.float32),
                "dof_pos.1": (q - default).astype(np.float32),
                "dof_vel.1": dq.astype(np.float32),
                "prev_actions.1": prev_actions.astype(np.float32),
                "projected_gravity.1": gvec.astype(np.float32),
                "buffer.1": hist_obs.astype(np.float32),
            }

            # Run policy
            policy_output = policy(input_data)
            target_q = policy_output["actions_scaled"]
            prev_actions = policy_output["actions"]
            hist_obs = policy_output["x.3"]

            # Apply commands with offset compensation
            commands = []
            for i, joint_name in enumerate(JOINT_NAME_LIST):
                joint_id = JOINT_NAME_TO_ID[joint_name]
                position = robot_state.apply_command(float(target_q[i] + default[i]), joint_name)
                commands.append({"actuator_id": joint_id, "position": position})
            await kos.actuator.command_actuators(commands)

            sleep = model_info["policy_dt"] - (time.time() - process_start)
            if sleep > 0:
                await asyncio.sleep(sleep)
            else:
                print(f"Policy took {time.time() - process_start} seconds")

            count_policy += 1

    except KeyboardInterrupt:
        print("\nStopping walking...")
    finally:
        # Disable torque on exit
        for joint_id in leg_ids:
            await kos.actuator.configure_actuator(actuator_id=joint_id, torque_enabled=False)

async def main():
    parser = argparse.ArgumentParser(description="Real robot walking deployment script.")
    parser.add_argument("--load_model", type=str, required=True, help="Path to policy model")
    parser.add_argument("--keyboard_use", action="store_true", help="Enable keyboard control")
    parser.add_argument("--robot_ip", type=str, default="100.69.185.128", help="Robot IP address")
    args = parser.parse_args()

    # Initialize KOS and policy
    kos = KOS(ip=args.robot_ip)
    policy = ONNXModel(args.load_model)
    
    # Get model info from policy metadata
    metadata = policy.get_metadata()
    model_info = {
        "num_actions": metadata["num_actions"],
        "num_observations": metadata["num_observations"],
        "robot_effort": metadata["robot_effort"],
        "robot_stiffness": metadata["robot_stiffness"],
        "robot_damping": metadata["robot_damping"],
        "tau_factor": metadata["tau_factor"],
        "policy_dt": metadata["sim_dt"] * metadata["sim_decimation"],
    }

    breakpoint()

    # Initialize velocity commands
    global x_vel_cmd, y_vel_cmd, yaw_vel_cmd
    if args.keyboard_use:
        x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.0, 0.0, 0.0
        pygame.init()
        pygame.display.set_caption("Robot Control")
    else:
        x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.5, 0.0, 0.0

    # Run robot control
    await run_robot(
        kos=kos,
        policy=policy,
        model_info=model_info,
        keyboard_use=args.keyboard_use,
    )

if __name__ == "__main__":
    asyncio.run(main())
