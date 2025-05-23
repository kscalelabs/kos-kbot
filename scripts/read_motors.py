import pykos
import asyncio
import os
import time

async def main():
    # Define the actuator IDs we want to monitor
    actuator_ids = list(range(11, 16)) + list(range(21, 26)) + list(range(31, 36)) + list(range(41, 46))
    async with pykos.KOS() as kos:
        while True:
            start_time = time.time()
            
            # Get states for all specified actuators
            state = await kos.actuator.get_actuators_state(actuator_ids)

            # Clear terminal (works on most terminals)
            os.system('cls' if os.name == 'nt' else 'clear')
            
            # Print dashboard header
            print("\n--- Actuator Dashboard ---")
            print("ID\t| Position")
            print("-" * 20)
            
            # Print each actuator's data
            for actuator_state in state.states:
                print(f"{actuator_state.actuator_id}\t| {actuator_state.position:.2f}")
            
            # Calculate sleep time to maintain 100Hz refresh rate
            elapsed = time.time() - start_time
            sleep_time = max(0, 0.1 - elapsed) 
            await asyncio.sleep(sleep_time)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nDashboard stopped.")
