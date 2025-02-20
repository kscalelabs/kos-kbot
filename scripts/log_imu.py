import pykos
import asyncio

async def main():
    csv_out = open("imu.csv", "w")
    async with pykos.KOS() as kos:
        try:
            while True:
                imu_data = await kos.imu.get_imu_values()
                gyro_x = imu_data.gyro_x or 0
                gyro_y = imu_data.gyro_y or 0
                gyro_z = imu_data.gyro_z or 0
                csv_out.write(f"{gyro_x}, {gyro_y}, {gyro_z}\n")
                print(f"gyro_x: {gyro_x}, gyro_y: {gyro_y}, gyro_z: {gyro_z}")
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            csv_out.close()

if __name__ == "__main__":
    asyncio.run(main())
