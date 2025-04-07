"""Script to enable or disable torque on robot actuators."""

import argparse
import asyncio
import logging

import colorlogging
from pykos import KOS

logger = logging.getLogger(__name__)
actuator_ids = [21, 22, 23, 24, 25, 11, 12, 13, 14, 15, 31, 32, 33, 34, 35, 41, 42, 43, 44, 45]  # shoulder pitch, roll, yaw, elbow, wrist

async def configure_actuator_torque(kos: KOS, actuator_id: int, enable: bool) -> bool:
    """Configure torque for a single actuator with error handling.
    
    Args:
        kos: KOS client instance
        actuator_id: ID of the actuator to configure
        enable: True to enable torque, False to disable
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=50.0,
            kd=5.0,
            max_torque=30,
            torque_enabled=enable
        )
        return True
    except Exception as e:
        logger.error(f"Failed to {'enable' if enable else 'disable'} torque for actuator {actuator_id}: {e}")
        return False

async def test_client(host: str = "localhost", port: int = 50051, enable_torque: bool = True) -> None:
    action = "Enabling" if enable_torque else "Disabling"
    logger.info(f"{action} torque on {host}:{port}...")
    
    async with KOS(ip=host, port=port) as kos:
        successful = 0
        failed = 0
        
        for actuator_id in actuator_ids:
            if await configure_actuator_torque(kos, actuator_id, enable_torque):
                successful += 1
            else:
                failed += 1
        
        logger.info(f"Torque {'enabled' if enable_torque else 'disabled'} for {successful} actuators. Failed for {failed} actuators.")

async def main() -> None:
    """Runs the main script."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost", help="Robot IP address")
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--disable", action="store_true", help="Disable torque (default is to enable)")
    args = parser.parse_args()

    colorlogging.configure(level=logging.DEBUG if args.debug else logging.INFO)
    await test_client(host=args.host, port=args.port, enable_torque=not args.disable)

if __name__ == "__main__":
    asyncio.run(main()) 