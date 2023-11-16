from mpu6050 import mpu6050
from time import sleep

sensor = mpu6050(0x68)
while True:
    gyro_data = sensor.get_gyro_data()
    accel_data = sensor.get_accel_data()
    temp = sensor.get_temp()
    print("GYR x" + "%7.2f" %  gyro_data['x'] + " y" + "%7.2f" %  gyro_data['y'] + " z" + "%7.2f" %  gyro_data['z'], end=' ')
    print("ACC X" + "%7.2f" % accel_data['x'] + " Y" + "%7.2f" % accel_data['y'] + " Z" + "%7.2f" % accel_data['z'], end=' ')
    print("TEMP " + "%4.1f" % temp + "C")
    sleep(0.2)
