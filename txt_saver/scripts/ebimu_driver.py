#!/usr/bin/env python3
#kroad-chson 230310

import rospy
import serial
import struct
import time, os

from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
from pyfiglet import Figlet

# font examples are here : http://www.figlet.org/examples.html
f = Figlet(font='larry3d')


port = rospy.get_param('port','/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0001-if00-port0')
baud = rospy.get_param('baud','921600')

# Initialize the node
rospy.init_node('imu_publisher')

# Set up the IMU message
imu_msg = Imu()
imu_msg.header = Header()
imu_msg.header.frame_id = 'imu_link'
imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

# Main loop
while not rospy.is_shutdown():
    # Set up the serial port and message publisher
    ser = None

    try:
        rospy.logdebug('ebimu_node')
        ser = serial.Serial(port, baud)

        while not rospy.is_shutdown():
            # Read data from the serial port
            rospy.logdebug_once('trying to connect IMU')

            data = ser.readline().decode().strip().split(',')

            # # Unpack the data into the IMU message
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w, \
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, \
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z, \
            temp, battery = map(float, data[1:])

            imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.x *9.81 # g unit to m/s^2
            imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.y *9.81 # g unit to m/s^2
            imu_msg.linear_acceleration.z = imu_msg.linear_acceleration.z *9.81 # g unit to m/s^2

            os.system('cls' if os.name == 'nt' else 'clear')
            print(f.renderText('LOCAL'))

            print('/////////////esbimu_driver/////////////\n\n')
            print(f'Quat : z:{imu_msg.orientation.x:.3f} | y:{imu_msg.orientation.y:.3f} | x:{imu_msg.orientation.z:.3f} | w:{imu_msg.orientation.w:.3f}',end='\n\n')
            from tf.transformations import euler_from_quaternion

            e = euler_from_quaternion([imu_msg.orientation.w,imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z])
            r = np.rad2deg(e[0])
            p = np.rad2deg(e[1])
            y = np.rad2deg(e[2])

            print(f'{r=}  {p=} {y=}')
            
            print(f'Accel: x:{imu_msg.linear_acceleration.x:.3f} | y:{imu_msg.linear_acceleration.y:.3f} | z:{imu_msg.linear_acceleration.z:.3f} m/s^2',end='\n\n')
            print(f'Gyro: x:{imu_msg.angular_velocity.x:.3f} | y:{imu_msg.angular_velocity.y:.3f} | z:{imu_msg.angular_velocity.z:.3f}deg/s',end='\n\n')
            print(f'imu_temp : {temp}C | battery : {battery}%',end='\n\n')


            # Publish the message
            imu_pub.publish(imu_msg)

    except serial.SerialException:
        rospy.logwarn(f'Unable to connect to serial port:{port}, baud:{baud}, retrying...')
        time.sleep(1.0)

