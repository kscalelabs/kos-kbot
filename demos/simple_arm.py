"""Interactive example script for making the robot wave."""

import argparse
import asyncio
import logging
import time

import colorlogging
import numpy as np
from pykos import KOS

logger = logging.getLogger(__name__)

async def move_to_target(kos: KOS, current_angle: float, target: float, 
                        acceleration: float, V_MAX: float, dt: float, 
                        actuator_id: list) -> float:
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
                'velocity': abs(target_velocity)  # velocity should always be positive
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


async def wave_sequence(kos: KOS) -> None:
    """Execute the waving sequence."""
    # Motion parameters
    V_MAX = 70.0  # deg/s - reduced from 50.0 for smoother motion
    ACCEL = 110.0  # deg/s² - reduced from 80.0 for smoother acceleration
    dt = 1/50.0  # seconds
    
    # Configure right arm actuators with softer gains
    right_arm_ids = [21, 22, 23, 24, 25]  # shoulder pitch, roll, yaw, elbow, wrist
    for actuator_id in right_arm_ids:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=50.0,  # reduced from 200.0
            kd=7.0,    # increased from 5.0 for better damping
            max_torque=85,
            torque_enabled=True
        )
    
    # Wave sequence with pauses between movements
    await move_to_target(kos, 0.0, 90.0, ACCEL, V_MAX, dt, 24)  # shoulder pitch up
    await move_to_target(kos, 90.0, 0.0, ACCEL, V_MAX, dt, 24)
    


async def test_client(host: str = "localhost", port: int = 50051) -> None:
    logger.info("Starting wave demo...")

    async with KOS(ip=host, port=port) as kos:
        # Remove the sim.reset call since it's not implemented
        await wave_sequence(kos)


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