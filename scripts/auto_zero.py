import math
import pykos
import asyncio
from dataclasses import dataclass

@dataclass
class MotorInfo:
    name: str
    motor_id: int
    offset: float
    kp: float
    kd: float
    max_torque: float

MOTORS = [
    # Right arm motors
    # MotorInfo(name="right_shoulder_pitch_03", motor_id=21, offset=2.094395, kp=40.0, kd=4.0, max_torque=10.0),
    # MotorInfo(name="right_shoulder_roll_03", motor_id=22, offset=-1.658063, kp=40.0, kd=4.0, max_torque=10.0),
    # MotorInfo(name="right_shoulder_yaw_02", motor_id=23, offset=1.658063, kp=30.0, kd=1.0, max_torque=3.0),
    # MotorInfo(name="right_elbow_02", motor_id=24, offset=2.478368, kp=30.0, kd=1.0, max_torque=3.0),
    # MotorInfo(name="right_wrist_02", motor_id=25, offset=1.745329, kp=20.0, kd=0.45473329537059787, max_torque=3.0),
    
    # Left arm motors
    # MotorInfo(name="left_shoulder_pitch_03", motor_id=11, offset=-1.047198, kp=40.0, kd=4.0, max_torque=10.0),
    MotorInfo(name="left_shoulder_roll_03", motor_id=12, offset=-0.383972, kp=60.0, kd=4.0, max_torque=10.0),
    MotorInfo(name="left_shoulder_yaw_02", motor_id=13, offset=-1.658063, kp=30.0, kd=1.0, max_torque=3.0),
    MotorInfo(name="left_elbow_02", motor_id=14, offset=-2.478368, kp=30.0, kd=1.0, max_torque=3.0),
    # MotorInfo(name="left_wrist_02", motor_id=15, offset=-1.745329, kp=20.0, kd=0.45473329537059787, max_torque=3.0),
    
    # Right leg motors
    MotorInfo(name="right_hip_pitch_04", motor_id=41, offset=1.047198, kp=85.0, kd=5.0, max_torque=20.0),
    # MotorInfo(name="right_hip_roll_03", motor_id=42, offset=0.209440, kp=40.0, kd=4.0, max_torque=10.0),
    MotorInfo(name="right_hip_yaw_03", motor_id=43, offset=-1.570796, kp=40.0, kd=4.0, max_torque=10.0),
    MotorInfo(name="right_knee_04", motor_id=44, offset=-2.705260, kp=85.0, kd=5.0, max_torque=20.0),
    MotorInfo(name="right_ankle_02", motor_id=45, offset=0.785398, kp=30.0, kd=1.0, max_torque=3.0),
    
    # Left leg motors
    MotorInfo(name="left_hip_pitch_04", motor_id=31, offset=2.216568, kp=85.0, kd=5.0, max_torque=20.0),
    MotorInfo(name="left_hip_roll_03", motor_id=32, offset=2.268928, kp=40.0, kd=4.0, max_torque=10.0),
    MotorInfo(name="left_hip_yaw_03", motor_id=33, offset=-1.570796, kp=40.0, kd=4.0, max_torque=10.0),
    MotorInfo(name="left_knee_04", motor_id=34, offset=2.705260, kp=85.0, kd=5.0, max_torque=20.0),
    MotorInfo(name="left_ankle_02", motor_id=35, offset=0.296706, kp=30.0, kd=1.0, max_torque=3.0),
]

async def main():
    with pykos.KOS() as kos:
        actuator_service = kos.actuator

        try:
        
            for motor in MOTORS:
                print(f"Zeroing motor: {motor.name} (ID: {motor.motor_id})")
                
                zero_position = motor.offset
                
                try:
                    # Zero the actuator against an endstop, then move back by the offset
                    await actuator_service.zero_actuators(
                        actuator_id=motor.motor_id,
                        endstop_position=math.degrees(zero_position),
                        configure_actuator={
                            "actuator_id": motor.motor_id,
                            "kp": motor.kp,
                            "kd": motor.kd,
                            "max_torque": motor.max_torque,
                        },
                        target_velocity=10.0,
                        commands_per_second=10,
                        move_back_seconds=10.0,
                    )
                    print(f"Successfully zeroed {motor.name}")
                except Exception as e:
                    print(f"Error zeroing {motor.name}: {e}")
                
                # Small delay between motors
                await asyncio.sleep(0.5)
            
            print("All motors zeroed successfully!")
        except asyncio.CancelledError:
            print("Zeroing cancelled")
            for motor in MOTORS:
                await actuator_service.configure_actuator(
                    actuator_id=motor.motor_id,
                    torque_enabled=False,
                )

if __name__ == "__main__":
    asyncio.run(main())
