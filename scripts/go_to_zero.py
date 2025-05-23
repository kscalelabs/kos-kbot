import time
import pykos
import asyncio

times = []
async def main():
    actuator_ids = list(range(11, 16)) + list(range(21, 26)) + list(range(31, 36)) + list(range(41, 46))
    async with pykos.KOS() as kos:
        # configure all actuators
        for id in actuator_ids:
            try:
                await kos.actuator.configure_actuator(actuator_id=id, kp=50, kd=5, torque_enabled=True)
            except Exception as e:
                print(f"Failed to configure actuator {id}")
        await asyncio.sleep(1)

        commands = [{'actuator_id': id, 'position': 0.0, 'velocity': 10.0} for id in range(60)]
        while True:
            print(f"Actuator command sent at: {time.time() * 1000}ms")
            await kos.actuator.command_actuators(commands)
            await asyncio.sleep(0.01)

if __name__ == "__main__":
    start_time = time.time()
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        print(f"Total time taken: {time.time() - start_time} seconds")
        print(f"Average time taken: {sum(times) / len(times)} seconds")
        print(f"Min time taken: {min(times)} seconds")
        print(f"Max time taken: {max(times)} seconds")
        print(f"Median time taken: {sorted(times)[len(times) // 2]} seconds")
        import numpy as np
        np_times = np.array(times)
        np.save('command_actuators_3.npy', np_times)
        print(f"Times saved to go_to_zero_times.npy")
