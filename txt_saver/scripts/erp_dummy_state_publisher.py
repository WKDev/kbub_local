#!/usr/bin/env python3
#kroad-chson 230310

import rospy, math
from std_msgs.msg import Header
from erp42_msgs.msg import SerialFeedBack
from pyfiglet import Figlet
import numpy as np

# font examples are here : http://www.figlet.org/examples.html
f = Figlet(font='larry3d')

port = rospy.get_param('port','/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0')
baud = rospy.get_param('baud','115200')

# Initialize the node
rospy.init_node('erp_state_simulator')

serial_pub = rospy.Publisher('erp42_serial', SerialFeedBack, queue_size=10)
feedback_msg = SerialFeedBack()

i = 0
# Main loop
while not rospy.is_shutdown():
    i = i+1
    feedback_msg.brake=0
    feedback_msg.encoder=i
    feedback_msg.steer=np.deg2rad(20*math.sin(i*0.1))
    feedback_msg.EStop=0
    feedback_msg.MorA=1
    feedback_msg.Gear=1
    feedback_msg.speed=0.1
    serial_pub.publish(feedback_msg)
    rospy.sleep(0.02)
rospy.spin()