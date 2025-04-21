import pykos
import time
import argparse
import matplotlib.pyplot as plt
from collections import deque
import csv
from datetime import datetime
import asyncio
import signal

kbot_v2 = "localhost"

ip_aliases = {
    "kbot-v2": kbot_v2
}

parser = argparse.ArgumentParser()
parser.add_argument('--plot', action='store_true', help='Plot the IMU data instead of printing')
parser.add_argument('--log', action='store_true', help='Save IMU data to a CSV file')
parser.add_argument('--ipalias', type=str, default='kbot-v2', help='IP alias of the KOS')
args = parser.parse_args()

async def main():
    # CSV file handle will be defined in global scope to access from signal handler
    csv_file = None
    
    try:
        # Use the context manager to properly initialize and close the connection
        async with pykos.KOS(ip=ip_aliases[args.ipalias]) as kos:
            imu = kos.imu

            if args.log:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                csv_filename = f'imu_log_{timestamp}.csv'
                csv_file = open(csv_filename, 'w', newline='')
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(['timestamp', 
                                    'accel_x (m/s^2)', 'accel_y (m/s^2)', 'accel_z (m/s^2)',
                                    'gyro_x (deg/s)', 'gyro_y (deg/s)', 'gyro_z (deg/s)',
                                    'mag_x (uT)', 'mag_y (uT)', 'mag_z (uT)',
                                    'roll (deg)', 'pitch (deg)', 'yaw (deg)',
                                    'quat_w', 'quat_x', 'quat_y', 'quat_z',
                                    'grav_x (m/s^2)', 'grav_y (m/s^2)', 'grav_z (m/s^2)'])

            if args.plot:
                plt.ion()
                fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1)
                
                history_len = 100
                accel_history = {'x': deque([0.0]*history_len, maxlen=history_len),
                                'y': deque([0.0]*history_len, maxlen=history_len),
                                'z': deque([0.0]*history_len, maxlen=history_len)}

                gyro_history = {'x': deque([0.0]*history_len, maxlen=history_len),
                              'y': deque([0.0]*history_len, maxlen=history_len),
                              'z': deque([0.0]*history_len, maxlen=history_len)}

                mag_history = {'x': deque([0.0]*history_len, maxlen=history_len),
                             'y': deque([0.0]*history_len, maxlen=history_len),
                             'z': deque([0.0]*history_len, maxlen=history_len)}
                
                time_history = deque(range(history_len), maxlen=history_len)

                euler_history = {'roll': deque([0.0]*history_len, maxlen=history_len),
                               'pitch': deque([0.0]*history_len, maxlen=history_len),
                               'yaw': deque([0.0]*history_len, maxlen=history_len)}

                quat_history = {'w': deque([0.0]*history_len, maxlen=history_len),
                              'x': deque([0.0]*history_len, maxlen=history_len),
                              'y': deque([0.0]*history_len, maxlen=history_len),
                              'z': deque([0.0]*history_len, maxlen=history_len)}

                gravity_history = {'x': deque([0.0]*history_len, maxlen=history_len),
                                 'y': deque([0.0]*history_len, maxlen=history_len),
                                 'z': deque([0.0]*history_len, maxlen=history_len)}

            while True:
                # Use await with all the async methods
                imu_values = await imu.get_imu_values()
                euler_angles = await imu.get_euler_angles()
                quaternion = await imu.get_quaternion()
                imu_advanced_values = await imu.get_imu_advanced_values()

                try:
                    grav_x = imu_advanced_values.grav_x
                    grav_y = imu_advanced_values.grav_y
                    grav_z = imu_advanced_values.grav_z
                except Exception as e:
                    print(f"Error reading gravity values: {e}")
                    grav_x = grav_y = grav_z = 0.0

                try:
                    roll = euler_angles.roll
                    pitch = euler_angles.pitch
                    yaw = euler_angles.yaw
                except Exception as e:
                    print(f"Error reading euler angles: {e}")
                    pitch = roll = yaw = 0.0

                try:
                    accel_x = imu_values.accel_x
                    accel_y = imu_values.accel_y
                    accel_z = imu_values.accel_z
                except Exception as e:
                    print(f"Error reading accelerometer values: {e}")
                    accel_x = accel_y = accel_z = 0.0

                try:
                    gyro_x = imu_values.gyro_x
                    gyro_y = imu_values.gyro_y
                    gyro_z = imu_values.gyro_z
                except Exception as e:
                    print(f"Error reading gyroscope values: {e}")
                    gyro_x = gyro_y = gyro_z = 0.0

                try:
                    mag_x = imu_values.mag_x
                    mag_y = imu_values.mag_y
                    mag_z = imu_values.mag_z
                except Exception as e:
                    print(f"Error reading magnetometer values: {e}")
                    mag_x = mag_y = mag_z = 0.0

                try:
                    quat_w = quaternion.w
                    quat_x = quaternion.x
                    quat_y = quaternion.y
                    quat_z = quaternion.z
                except Exception as e:
                    print(f"Error reading quaternion values: {e}")
                    quat_w = quat_x = quat_y = quat_z = 0.0

                # Round to make not bad
                accel_x = format(round(accel_x, 10), '.10f')
                accel_y = format(round(accel_y, 10), '.10f')
                accel_z = format(round(accel_z, 10), '.10f')

                gyro_x = format(round(gyro_x, 10), '.10f')
                gyro_y = format(round(gyro_y, 10), '.10f')
                gyro_z = format(round(gyro_z, 10), '.10f')

                mag_x = format(round(mag_x, 10), '.10f')
                mag_y = format(round(mag_y, 10), '.10f')
                mag_z = format(round(mag_z, 10), '.10f')

                roll = format(round(roll, 10), '.10f')
                pitch = format(round(pitch, 10), '.10f')
                yaw = format(round(yaw, 10), '.10f')

                quat_w = format(round(quat_w, 10), '.10f')
                quat_x = format(round(quat_x, 10), '.10f')
                quat_y = format(round(quat_y, 10), '.10f')
                quat_z = format(round(quat_z, 10), '.10f')

                # Round gravity values
                grav_x = format(round(grav_x, 10), '.10f')
                grav_y = format(round(grav_y, 10), '.10f')
                grav_z = format(round(grav_z, 10), '.10f')

                if args.plot:
                    # Update histories
                    accel_history['x'].append(float(accel_x))
                    accel_history['y'].append(float(accel_y))
                    accel_history['z'].append(float(accel_z))
                    
                    gyro_history['x'].append(float(gyro_x))
                    gyro_history['y'].append(float(gyro_y))
                    gyro_history['z'].append(float(gyro_z))

                    mag_history['x'].append(float(mag_x))
                    mag_history['y'].append(float(mag_y))
                    mag_history['z'].append(float(mag_z))
                    
                    time_history.append(time_history[-1] + 1)
                    
                    euler_history['roll'].append(float(roll))
                    euler_history['pitch'].append(float(pitch))
                    euler_history['yaw'].append(float(yaw))
                    
                    quat_history['w'].append(float(quat_w))
                    quat_history['x'].append(float(quat_x))
                    quat_history['y'].append(float(quat_y))
                    quat_history['z'].append(float(quat_z))
                    
                    gravity_history['x'].append(float(grav_x))
                    gravity_history['y'].append(float(grav_y))
                    gravity_history['z'].append(float(grav_z))
                    
                    ax1.clear()
                    ax2.clear()
                    ax3.clear()
                    ax4.clear()
                    ax5.clear()
                    ax6.clear()
                    
                    # accel
                    ax1.plot(time_history, accel_history['x'], label='X')
                    ax1.plot(time_history, accel_history['y'], label='Y')
                    ax1.plot(time_history, accel_history['z'], label='Z')
                    ax1.set_title('Accelerometer')
                    ax1.legend()
                    
                    # gyro
                    ax2.plot(time_history, gyro_history['x'], label='X')
                    ax2.plot(time_history, gyro_history['y'], label='Y')
                    ax2.plot(time_history, gyro_history['z'], label='Z')
                    ax2.set_title('Gyroscope')
                    ax2.legend()

                    # mag
                    ax3.plot(time_history, mag_history['x'], label='X')
                    ax3.plot(time_history, mag_history['y'], label='Y')
                    ax3.plot(time_history, mag_history['z'], label='Z')
                    ax3.set_title('Magnetometer')
                    ax3.legend()

                    # Euler angles
                    ax4.plot(time_history, euler_history['roll'], label='Roll')
                    ax4.plot(time_history, euler_history['pitch'], label='Pitch')
                    ax4.plot(time_history, euler_history['yaw'], label='Yaw')
                    ax4.set_title('Euler Angles')
                    ax4.legend()

                    # Quaternion
                    ax5.plot(time_history, quat_history['w'], label='W')
                    ax5.plot(time_history, quat_history['x'], label='X')
                    ax5.plot(time_history, quat_history['y'], label='Y')
                    ax5.plot(time_history, quat_history['z'], label='Z')
                    ax5.set_title('Quaternion')
                    ax5.legend()

                    # Gravity
                    ax6.plot(time_history, gravity_history['x'], label='X')
                    ax6.plot(time_history, gravity_history['y'], label='Y')
                    ax6.plot(time_history, gravity_history['z'], label='Z')
                    ax6.set_title('Gravity')
                    ax6.legend()
                    
                    plt.pause(0.01)
                else:
                    pass
                    # print("\033[2J]\033[H")
                    # print(f"Accel: ({accel_x}, {accel_y}, {accel_z})")
                    # print(f"Gyro: ({gyro_x}, {gyro_y}, {gyro_z})")
                    # print(f"Mag: ({mag_x}, {mag_y}, {mag_z})")
                    # print(f"Euler: (roll: {roll}, pitch: {pitch}, yaw: {yaw})")
                    # print(f"Quaternion: (w: {quat_w}, x: {quat_x}, y: {quat_y}, z: {quat_z})")
                    # print(f"Gravity: ({grav_x}, {grav_y}, {grav_z})")

                # Log to CSV if enabled
                if args.log:
                    csv_writer.writerow([datetime.now().isoformat(),
                                       accel_x, accel_y, accel_z,
                                       gyro_x, gyro_y, gyro_z,
                                       mag_x, mag_y, mag_z,
                                       roll, pitch, yaw,
                                       quat_w, quat_x, quat_y, quat_z,
                                       grav_x, grav_y, grav_z])
                    csv_file.flush()

                # Use asyncio.sleep instead of time.sleep in async functions
                # await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user (Ctrl+C)")
        if args.log and csv_file:
            print("Saving CSV file and closing...")
            csv_file.close()
        print("Exiting gracefully")
    except Exception as e:
        print(f"An error occurred: {e}")
        if args.log and csv_file:
            csv_file.close()
    finally:
        # Ensure file is closed in any case
        if args.log and csv_file:
            csv_file.close()

# Run the async main function
if __name__ == "__main__":
    asyncio.run(main())