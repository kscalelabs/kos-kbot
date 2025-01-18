import pykos
import time
kos = pykos.KOS()

imu = kos.imu

while True:
    try:
        print(imu.get_imu_values())
    except Exception as e:
        print(e)
    time.sleep(0.1)
