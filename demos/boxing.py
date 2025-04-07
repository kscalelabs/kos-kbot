"""Interactive example script for making the robot take a boxing stance."""

import argparse
import asyncio
import logging
import time

import colorlogging
import numpy as np
from pykos import KOS

logger = logging.getLogger(__name__)

# Define actuator IDs for both hands
right_hand_ids = [51, 52, 53, 54, 55]
left_hand_ids = [56, 57, 58, 59, 60]

async def move_hands(kos: KOS, hand_ids: list[int], position: int) -> None:
    """Move hand actuators to specified position.
    
    Args:
        kos: KOS client instance
        hand_ids: List of actuator IDs to move
        position: Target position (0 or 1, where 0=0deg and 1=100deg)
    """
    if position not in [0, 1]:
        raise ValueError("Position must be 0 or 1")
        
    target = 100.0 if position == 1 else 0.0
    
    # Configure actuators
    for actuator_id in hand_ids:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=10.0,
            kd=5.0,
            max_torque=30,
            torque_enabled=True
        )
    
    # Move to target position
    commands = [
        {
            'actuator_id': id,
            'position': target,
            'velocity': 0.0 
        }
        for id in hand_ids
    ]
    await kos.actuator.command_actuators(commands)

async def move_to_target(kos: KOS, current_angle: float, target: float, 
                        acceleration: float, V_MAX: float, dt: float, 
                        actuator_id: int) -> float:
    """Move a single actuator to target position with controlled velocity profile."""
    # Get actual current position from the robot
    response = await kos.actuator.get_actuators_state([actuator_id])
    current_angle = response.states[0].position
    current_velocity = response.states[0].velocity

    displacement = target - current_angle
    distance = abs(displacement)
    direction = 1 if displacement > 0 else -1
    
    # Calculate time needed to reach max velocity
    t_accel = V_MAX / acceleration
    d_accel = 0.5 * acceleration * t_accel**2
    
    if distance >= 2 * d_accel:
        # Trapezoidal profile
        d_flat = distance - 2 * d_accel
        t_flat = d_flat / V_MAX
        total_time = 2 * t_accel + t_flat

        def get_target_velocity(t):
            if t < t_accel:  # Acceleration phase
                return direction * min(V_MAX, current_velocity + acceleration * t)
            elif t < (t_accel + t_flat):  # Constant velocity phase
                return direction * V_MAX
            else:  # Deceleration phase
                return direction * max(0, V_MAX - acceleration * (t - t_accel - t_flat))
    else:
        # Triangular profile
        t_peak = (distance / acceleration) ** 0.5
        total_time = 2 * t_peak
        peak_velocity = acceleration * t_peak

        def get_target_velocity(t):
            if t < t_peak:  # Acceleration phase
                return direction * min(peak_velocity, current_velocity + acceleration * t)
            else:  # Deceleration phase
                return direction * max(0, peak_velocity - acceleration * (t - t_peak))
    
    steps = int(total_time / dt)
    next_tick = time.perf_counter()
    t = 0.0
    
    for _ in range(steps):
        target_velocity = get_target_velocity(t)
        current_angle += target_velocity * dt
        
        await kos.actuator.command_actuators([
            {
                'actuator_id': actuator_id,
                'position': current_angle,
                'velocity': abs(target_velocity)
            }
        ])
        
        t += dt
        next_tick += dt
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
    
    # Ensure we reach the final position with zero velocity
    await kos.actuator.command_actuators([
        {
            'actuator_id': actuator_id,
            'position': target,
            'velocity': 0.0
        }
    ])
    return target

async def boxing_stance(kos: KOS) -> None:
    """Execute the boxing stance sequence."""
    # Motion parameters
    V_MAX = 60.0  # deg/s
    ACCEL = 100.0  # deg/s²
    dt = 1/50.0  # seconds
    
    # Configure arm actuators with appropriate gains
    right_arm_ids = [21, 22, 23, 24, 25]  # Right: shoulder pitch, roll, yaw, elbow, wrist
    left_arm_ids = [11, 12, 13, 14, 15]   # Left: shoulder pitch, roll, yaw, elbow, wrist
    
    for actuator_id in right_arm_ids + left_arm_ids:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=50.0,
            kd=5.0,
            max_torque=50,
            torque_enabled=True
        )
    
    # Move to boxing stance - right arm forward, left arm guard
    await asyncio.gather(
        # Right arm (forward jab position)
        move_to_target(kos, 0.0, -40.0, ACCEL, V_MAX, dt, 21),  # shoulder pitch
        move_to_target(kos, 0.0, 5.0, ACCEL, V_MAX, dt, 22),  # shoulder roll
        move_to_target(kos, 0.0, -10.0, ACCEL, V_MAX, dt, 23),  # shoulder yaw
        move_to_target(kos, 0.0, 90.0, ACCEL, V_MAX, dt, 24),   # elbow
        move_to_target(kos, 0.0, 0.0, ACCEL, V_MAX, dt, 25),    # wrist
        
        # Left arm (guard position)
        move_to_target(kos, 0.0, 30.0, ACCEL, V_MAX, dt, 11),  # shoulder pitch
        move_to_target(kos, 0.0, -5.0, ACCEL, V_MAX, dt, 12),   # shoulder roll
        move_to_target(kos, 0.0, 10.0, ACCEL, V_MAX, dt, 13),   # shoulder yaw
        move_to_target(kos, 0.0, -100.0, ACCEL, V_MAX, dt, 14),  # elbow
        move_to_target(kos, 0.0, 0.0, ACCEL, V_MAX, dt, 15),    # wrist
        
        # Close fists
        move_hands(kos, right_hand_ids, 0),
        move_hands(kos, left_hand_ids, 0)
    )
    
    # Hold the pose for a moment
    await asyncio.sleep(3.0)
    
    # Return to neutral position
    await asyncio.gather(
    #     # Right arm
        move_to_target(kos, -40.0, 0.0, ACCEL, V_MAX, dt, 21),
        move_to_target(kos, 5.0, 0.0, ACCEL, V_MAX, dt, 22),
        move_to_target(kos, -10.0, 0.0, ACCEL, V_MAX, dt, 23),
        move_to_target(kos, 90.0, 0.0, ACCEL, V_MAX, dt, 24),
        move_to_target(kos, 0.0, 0.0, ACCEL, V_MAX, dt, 25),
        
    #     # Left arm
        move_to_target(kos, -30.0, 0.0, ACCEL, V_MAX, dt, 11),
        move_to_target(kos, -5.0, 0.0, ACCEL, V_MAX, dt, 12),
        move_to_target(kos, 10.0, 0.0, ACCEL, V_MAX, dt, 13),
        move_to_target(kos, -100.0, 0.0, ACCEL, V_MAX, dt, 14),
        move_to_target(kos, 0.0, 0.0, ACCEL, V_MAX, dt, 15),
        
    #     # Open hands
        move_hands(kos, right_hand_ids, 1),
        move_hands(kos, left_hand_ids, 1)
    )

async def test_client(host: str = "localhost", port: int = 50051) -> None:
    logger.info("Starting boxing stance demo...")

    async with KOS(ip=host, port=port) as kos:
        await boxing_stance(kos)

async def main() -> None:
    """Runs the main simulation loop."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    colorlogging.configure(level=logging.DEBUG if args.debug else logging.INFO)
    await test_client(host=args.host, port=args.port)

if __name__ == "__main__":
    asyncio.run(main())
