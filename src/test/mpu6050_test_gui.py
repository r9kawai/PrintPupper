from mpu6050 import mpu6050
from time import sleep
import numpy as np
import matplotlib.pyplot as plt

def plot_loop():
    sensor = mpu6050(0x68)

    # Get sensor data
    temp = "%4.1f" % sensor.get_temp()
    gyro_data = sensor.get_gyro_data()
    accel_data = sensor.get_accel_data()
    
    fig, (ax_gyro, ax_accel) = plt.subplots(ncols=2, figsize=(10,5))
    
    # All X axis = sec
    sec = np.arange(-np.pi, np.pi, 0.1)
    
    gyro_list_x = np.zeros(63)
    gyro_list_x[0] = "%6.3f" % gyro_data['x']
    gyro_x_lines, = ax_gyro.plot(sec, gyro_list_x, color="red", label="Pitch(x)")

    gyro_list_y = np.zeros(63)
    gyro_list_y[0] = "%6.3f" % gyro_data['y']
    gyro_y_lines, = ax_gyro.plot(sec, gyro_list_y, color="blue", label="Roll(y)")
    
    gyro_list_z = np.zeros(63)
    gyro_list_z[0] = "%6.3f" % gyro_data['z']
    gyro_z_lines, = ax_gyro.plot(sec, gyro_list_z, color="green", label="Yaw(z)")
    
    ax_gyro.legend()
    ax_gyro.set_title("Gyro x,y,z")
    ax_gyro.set_ylim(-300, 300)
    ax_gyro.set_xticks([])

    accel_list_x = np.zeros(63)
    accel_list_x[0] = "%6.3f" % accel_data['x']
    accel_x_lines, = ax_accel.plot(sec, accel_list_x, color="red", label="Right/Left(X)")

    accel_list_y = np.zeros(63)
    accel_list_y[0] = "%6.3f" % accel_data['y']
    accel_y_lines, = ax_accel.plot(sec, accel_list_y, color="blue", label="Forward/Back(Y)")
    
    accel_list_z = np.zeros(63)
    accel_list_z[0] = "%6.3f" % accel_data['z']
    accel_z_lines, = ax_accel.plot(sec, accel_list_z, color="green", label="Up/Down(Z)")
        
    ax_accel.legend()
    ax_accel.set_title("Accel X,Y,Z")
    ax_accel.set_ylim(-30, 30)
    ax_accel.set_xticks([])

    while True:
        temp = "%4.1f" % sensor.get_temp()
        gyro_data = sensor.get_gyro_data()
        accel_data = sensor.get_accel_data()
        
        sec += 0.1
      
        gyro_list_x = np.roll(gyro_list_x, 1)
        gyro_list_x[0] = "%6.3f" % gyro_data['x']
        gyro_list_y = np.roll(gyro_list_y, 1)
        gyro_list_y[0] = "%6.3f" % gyro_data['y']
        gyro_list_z = np.roll(gyro_list_z, 1)
        gyro_list_z[0] = "%6.3f" % gyro_data['z']

        accel_list_x = np.roll(accel_list_x, 1)
        accel_list_x[0] = "%6.3f" % accel_data['x']
        accel_list_y = np.roll(accel_list_y, 1)
        accel_list_y[0] = "%6.3f" % accel_data['y']
        accel_list_z = np.roll(accel_list_z, 1)
        accel_list_z[0] = "%6.3f" % accel_data['z']

        gyro_x_lines.set_data(sec, gyro_list_x)
        gyro_y_lines.set_data(sec, gyro_list_y)
        gyro_z_lines.set_data(sec, gyro_list_z)

        accel_x_lines.set_data(sec, accel_list_x)
        accel_y_lines.set_data(sec, accel_list_y)
        accel_z_lines.set_data(sec, accel_list_z)

        ax_gyro.set_xlim((sec.min(), sec.max()))
        ax_accel.set_xlim((sec.min(), sec.max()))

        print("GYR x" + "%7.2f" %  gyro_data['x'] + " y" + "%7.2f" %  gyro_data['y'] + " z" + "%7.2f" %  gyro_data['z'], end=' ')
        print("ACC X" + "%7.2f" % accel_data['x'] + " Y" + "%7.2f" % accel_data['y'] + " Z" + "%7.2f" % accel_data['z'], end=' ')
        print("TEMP " + temp + "C")
        plt.pause(0.1)

if __name__ == "__main__":
    plot_loop()
