#!/usr/bin/env python3
#kroad-chson 230310

import rospy
import serial
import struct
import time, os
import binascii
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from pyfiglet import Figlet

from erp42_msgs.msg import SerialFeedBack

# font examples are here : http://www.figlet.org/examples.html
f = Figlet(font='larry3d')


# port = rospy.get_param('port','/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0')
port = rospy.get_param('port','/dev/ttyUSB2')
baud = rospy.get_param('baud','115200')

# Initialize the node
rospy.init_node('pyERP_driver')
# rospy.init_node('imu_stat_publisher')

# Set up the IMU message
erp_msg = SerialFeedBack()
erp_pub = rospy.Publisher('erp42_serial', SerialFeedBack, queue_size=10)


# Main loop
while not rospy.is_shutdown():
    # Set up the serial port and message publisher
    ser = None

    try:
        rospy.loginfo('pyERP_driver')
        ser = serial.Serial(port, baud)
        time.sleep(0.5)
        
        while not rospy.is_shutdown():
            # Read data from the serial port
            rospy.loginfo_once('trying to connect erp')

            data = ser.readline()

            # print(data)

            # data = b'STX\x00\x00\x00\x00\x00\xa2\xff\x01\x19\x00\x00\x00R\r\n'
            # i found much more easier way to parse this....
            # data came from serial binary has a form of bytearray.
            hex_string = binascii.hexlify(data).decode('utf-8')
            pair_list = [hex_string[i:i+2] for i in range(0, len(hex_string), 2)][3:-2]

            os.system('cls' if os.name == 'nt' else 'clear')
            print(f.renderText('LOCAL'))

            if len(pair_list)>8:
                m_or_a = int(pair_list[0],16)
                e_stop = int(pair_list[1],16)
                gear = int(pair_list[2],16)
                speed = int(''.join(pair_list[3:4][::-1]),16)*10
                steer = int(''.join(pair_list[5:7][::-1]),16)

                if steer > 30000:
                    steer = steer - 65536

                steer = -steer/71
                break_ = int(pair_list[7],16)
                enc = int(''.join(pair_list[8:-1][::-1]),16)
                alive = int(pair_list[-1],16)
                
                print(f'{m_or_a=} {e_stop=} {gear=} {speed=} {steer=:.2f} {break_=} {enc=} {alive=}')

                erp_msg.alive = alive
                erp_msg.brake = break_
                erp_msg.Gear = gear
                erp_msg.MorA = m_or_a
                erp_msg.EStop = e_stop
                erp_msg.speed = speed
                erp_msg.steer = steer
                erp_msg.encoder = enc

                erp_pub.publish(erp_msg)

    except serial.SerialException:
        rospy.logwarn(f'Unable to connect to serial port:{port}, baud:{baud}, retrying...')
        time.sleep(1.0)

