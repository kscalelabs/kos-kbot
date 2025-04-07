"""Interactive example script for making the robot wave."""

import argparse
import asyncio
import logging
import time
from typing import Union

import colorlogging
import numpy as np
from pykos import KOS

logger = logging.getLogger(__name__)
right_hand_ids = [51, 52, 53, 54, 55, 61, 62, 63, 64, 65, 66]
actuator_ids =  right_hand_ids


async def wave_sequence(kos: KOS) -> None:
    """Execute the waving sequence."""
    for actuator_id in actuator_ids:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=10.0,  # reduced from 200.0
            kd=5.0,    # increased from 5.0 for better damping
            max_torque=30,
            torque_enabled=True
        )
        
    await kos.actuator.command_actuators(
        [
            {
                'actuator_id': aid,
                'position': 0.0,
                'velocity': 100.0 
            }
            for aid in actuator_ids
        ]
    )

    await asyncio.sleep(3.0)
    await kos.actuator.command_actuators(
        [
            {
                'actuator_id': aid,
                'position': 100.0,
                'velocity': 0.0 
            }
            for aid in actuator_ids
        ]
    )
    
async def test_client(host: str = "localhost", port: int = 50051) -> None:
    logger.info("Starting wave demo...")

    async with KOS(ip=host, port=port) as kos:
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