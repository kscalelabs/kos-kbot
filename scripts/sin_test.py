"""Script to stress test the leg motors with a triangle wave.

This script will configure the leg motors with the appropriate gains and then move them in a triangle wave motion.

Currently, it semes to die at 10hz and 20 deg amplitude.
"""

import pykos
import asyncio
import argparse
import math
import time

# Define leg motor IDs and their names for reference
LEG_MOTORS = {
    # Left leg
    "left_hip_pitch": 31,
    "left_hip_roll": 32,
    "left_hip_yaw": 33,
    "left_knee": 34,
    "left_ankle": 35,
    # Right leg
    "right_hip_pitch": 41,
    "right_hip_roll": 42,
    "right_hip_yaw": 43,
    "right_knee": 44,
    "right_ankle": 45
}

# Motor type groupings for different gains
R04_IDS = [31, 34, 41, 44]  # Stronger motors
R03_IDS = [32, 33, 42, 43]  # Medium motors
R02_IDS = [35, 45]          # Smaller motors

async def run_sine_test(kos: pykos.KOS, amplitude: float, frequency: float, duration: float) -> None:
    """
    Run triangle wave motion test on all leg motors.
    
    Args:
        kos: KOS instance
        amplitude: Motion amplitude in degrees
        frequency: Oscillation frequency in Hz
        duration: Test duration in seconds
    """
    # First disable all motors
    print("Disabling motors...")
    for motor_id in LEG_MOTORS.values():
        try:
            await kos.actuator.configure_actuator(
                actuator_id=motor_id,
                torque_enabled=False
            )
        except Exception as e:
            print(f"Failed to disable motor {motor_id}: {e}")
    
    await asyncio.sleep(1)
    
    # Configure motors with appropriate gains based on type
    print("Configuring motors...")
    for motor_id in LEG_MOTORS.values():
        try:
            if motor_id in R04_IDS:
                await kos.actuator.configure_actuator(
                    actuator_id=motor_id,
                    kp=250,
                    kd=5,
                    max_torque=80,
                    torque_enabled=True
                )
            elif motor_id in R03_IDS:
                await kos.actuator.configure_actuator(
                    actuator_id=motor_id,
                    kp=150,
                    kd=5,
                    max_torque=60,
                    torque_enabled=True
                )
            else:  # R02_IDS
                await kos.actuator.configure_actuator(
                    actuator_id=motor_id,
                    kp=40,
                    kd=5,
                    max_torque=17,
                    torque_enabled=True
                )
        except Exception as e:
            print(f"Failed to configure motor {motor_id}: {e}")
    
    print(f"Starting triangle wave test with:")
    print(f"  Amplitude: ±{amplitude}°")
    print(f"  Frequency: {frequency} Hz")
    print(f"  Duration: {duration} s")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time
        
        # Calculate triangle wave position
        # Period is 1/frequency, multiply by 4 because we need to complete full cycle
        # (0 -> +amp -> 0 -> -amp -> 0)
        period = 1.0 / frequency
        cycle_position = (t % period) / period

        _ = await kos.actuator.get_actuators_state(list(LEG_MOTORS.values()))
        
        if cycle_position < 0.25:  # Ramp up to positive
            position = (cycle_position * 4) * amplitude
        elif cycle_position < 0.5:  # Ramp down to zero
            position = (0.5 - cycle_position) * 4 * amplitude
        elif cycle_position < 0.75:  # Ramp down to negative
            position = ((cycle_position - 0.5) * 4) * -amplitude
        else:  # Ramp up to zero
            position = (cycle_position - 1.0) * 4 * amplitude
        
        # Prepare commands for all motors
        commands = [
            {"actuator_id": motor_id, "position": position}
            for motor_id in LEG_MOTORS.values()
        ]
        
        # Send commands
        try:
            await kos.actuator.command_actuators(commands)
            
            # Get current positions for monitoring
            states = await kos.actuator.get_actuators_state(list(LEG_MOTORS.values()))
            
            # Print status (clear line and move cursor up for clean output)
            print("\033[K", end="")  # Clear line
            print(f"Time: {t:.2f}s")
            print("\033[K", end="")  # Clear line
            print(f"Command: {position:.2f}°")
            for state in states.states:
                print("\033[K", end="")  # Clear line
                print(f"Motor {state.actuator_id}: {state.position:.2f}°")
            print("\033[F" * (len(states.states) + 2), end="")  # Move cursor up
            
        except Exception as e:
            print(f"Error during execution: {e}")
        
        await asyncio.sleep(0.01)  # 100Hz control rate
    
    print("\n" * (len(LEG_MOTORS) + 2))  # Clear status display
    print("Test complete. Disabling motors...")
    
    # Disable motors after test
    for motor_id in LEG_MOTORS.values():
        try:
            await kos.actuator.configure_actuator(
                actuator_id=motor_id,
                torque_enabled=False
            )
        except Exception as e:
            print(f"Failed to disable motor {motor_id}: {e}")

async def main() -> None:
    parser = argparse.ArgumentParser(description="Run sinusoidal test on leg motors")
    parser.add_argument("--amplitude", type=float, default=20.0,
                       help="Triangle wave amplitude in degrees (default: 20.0)")
    parser.add_argument("--frequency", type=float, default=0.5,
                       help="Triangle wave frequency in Hz (default: 0.5)")
    parser.add_argument("--duration", type=float, default=10.0,
                       help="Test duration in seconds (default: 10.0)")
    args = parser.parse_args()

    try:
        async with pykos.KOS() as kos:
            await run_sine_test(kos, args.amplitude, args.frequency, args.duration)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nTest interrupted! Disabling motors...")
        async with pykos.KOS() as kos:
            for motor_id in LEG_MOTORS.values():
                try:
                    await kos.actuator.configure_actuator(
                        actuator_id=motor_id,
                        torque_enabled=False
                    )
                except Exception as e:
                    print(f"Failed to disable motor {motor_id}: {e}")
        print("Motors disabled.")

if __name__ == "__main__":
    asyncio.run(main())
