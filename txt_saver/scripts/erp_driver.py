#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from erp42_msgs.msg import DriveCmd, ModeCmd, SerialFeedBack
import tf2_geometry_msgs
import tf2_ros
import math
import geometry_msgs
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler

import rospy
import math
from erp42_msgs.msg import CANFeedBack, SerialFeedBack
from nav_msgs.msg import Odometry

class OdometryManager:
    def __init__(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.wheel_base = 0.0
        self.steer_angle = 0.0
        self.delta_encoder = 0
        self.encoder = 0
        self.last_encoder = 0
        self.wheel_pos = 0.0
        self.ns_ = 'erp42_driver'

        self.wheel_radius = rospy.get_param('/my_robot/wheel_radius', 0.265)
        self.wheel_base = rospy.get_param('/my_robot/wheel_base', 1.040)
        self.wheel_tread = rospy.get_param('/my_robot/wheel_tread', 0.985)
        self.max_vel = rospy.get_param('/my_robot/max_vel', 5.0)
        self.min_vel = rospy.get_param('/my_robot/min_vel', -5.0)
        self.max_steer_angle = rospy.get_param('/my_robot/max_steer_angle', 28.169)
        self.min_steer_angle = rospy.get_param('/my_robot/min_steer_angle', -28.169)

        self.TICK2RAD = 0.06283185307

        # self.ns_ = rospy.get_namespace()

        self.odom = Odometry()

        self.sub = rospy.Subscriber('erp42_serial', SerialFeedBack, self.CalculateOdometry)


    def CalculateOdometry(self, data):
        # rospy.loginfo('calculating odometry....')
        # print(f'{self.delta_encoder=} {self.TICK2RAD=} {self.encoder=}')
        delta_time = 0.2
        if type(data) is not float:
            # 엔코더 변화량 측정
            self.delta_encoder = data.encoder - self.last_encoder
            self.last_encoder = data.encoder

            # 시간 주기동안 얼마나 굴러간 각도
            self.wheel_pos = self.TICK2RAD * self.delta_encoder

            # 각도 --> 길이(반지름* 각도)
            self.delta_pos = self.wheel_radius * self.wheel_pos

            # 선속도 
            self.linear_vel = self.delta_pos / delta_time
            
            # 각속도
            self.angular_vel = math.tan(data.steer) * self.linear_vel / self.wheel_base

            self.odom_yaw += self.angular_vel

            self.odom_x += self.delta_pos * math.cos(self.odom_yaw)
            self.odom_y += self.delta_pos * math.sin(self.odom_yaw)

            rospy.loginfo("Previous Encoder: %d", self.last_encoder)
            rospy.loginfo("Delta Pose: %f", self.delta_pos)
            rospy.loginfo("Linear Vel: %f", self.linear_vel)
            rospy.loginfo("Angular Vel: %f", self.angular_vel)
            rospy.loginfo("Odom X: %f", self.odom_x)
            rospy.loginfo("Odom Y: %f", self.odom_y)
            rospy.loginfo("Odom Yaw: %f", self.odom_yaw)

            if math.isnan(self.delta_pos):
                self.delta_pos = 0.0
            if math.isnan(self.angular_vel):
                self.angular_vel = 0.0

    
    def ResetOdometry(self):
        self.SetOdometry(0.0, 0.0, 0.0)

    def SetOdometry(self, new_x, new_y, new_yaw):
        self.odom_x = new_x
        self.odom_y = new_y
        self.odom_yaw = new_yaw
            

class ERP42Driver(object):
    def __init__(self):
        rospy.init_node('erp42_driver_py')
        self.erp42_interface_ = OdometryManager()
        self.rate_ = rospy.Rate(50)
        self.m_odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.m_current_time = rospy.Time.now()
        self.m_last_time = rospy.Time.now()
        self.m_last_odom_x = 0.0
        self.m_mode_MorA = 0
        self.m_mode_EStop = 0
        self.m_mode_Gear = 0

        self.InitParam()
        self.InitNode()
        self.erp42_interface_.ResetOdometry()
        self.MAX_KPH = 5
        self.MAX_DEGREE = 20



         # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create a tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Create an Odometry publisher
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)

        # Create a timer to update the Odometry message
        self.timer = rospy.Timer(rospy.Duration(0.1), self.UpdateOdometry)


    def MIN(self, a,b):
        if a>b:
            return b
        else:
            return a
        
    def MAX(self,a,b):
        if a<b:
            return b
        else:
            return a
        
    def mps_to_kph(self,a):
        return a*3.6


    def InitParam(self):
        # DEBUG MODE
        if rospy.get_param('/rosout_level', 'INFO') == 'DEBUG':
            rospy.logdebug('Debug mode enabled')

        # Initialize [m]
        self.wheel_base = 1.040
        self.wheel_radius = 0.265
        self.wheel_tread = 0.985
        self.max_vel = 5.0
        self.min_vel = -5.0
        self.max_steer_angle = 28.169
        self.min_steer_angle = -28.169

        self.wheel_radius = rospy.get_param('wheel_radius', self.wheel_radius)
        self.wheel_base = rospy.get_param('wheel_base', self.wheel_base)
        self.wheel_tread = rospy.get_param('wheel_tread', self.wheel_tread)
        self.max_vel = rospy.get_param('max_vel', self.max_vel)
        self.min_vel = rospy.get_param('min_vel', self.min_vel)
        self.max_steer_angle = rospy.get_param('max_steer_angle', self.max_steer_angle)
        self.min_steer_angle = rospy.get_param('min_steer_angle', self.min_steer_angle)

    def InitNode(self):
        self.m_pub_odom = rospy.Publisher(self.erp42_interface_.ns_+'/odom', Odometry, queue_size=1)
        self.m_pub_drive = rospy.Publisher(self.erp42_interface_.ns_+'/drive', DriveCmd, queue_size=1)
        self.m_pub_mode = rospy.Publisher(self.erp42_interface_.ns_+'/mode', ModeCmd, queue_size=1)
        self.m_sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.CmdVelCallback)
        self.m_sub_mode = rospy.Subscriber(self.erp42_interface_.ns_+'/without_gear_mode', ModeCmd, self.ModeCallback)

    def CmdVelCallback(self, msg):
        # m/s to KPH
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        radius = linear_vel / angular_vel
        steering_angle = math.atan(self.wheel_base / radius)

        if linear_vel == 0.0 or angular_vel == 0.0:
            radius = 0.0
            steering_angle = 0.0

        # m_mode_Gear = GEAR.FORWARD.value if linear_vel >= 0 else GEAR.REVERSE.value
        m_mode_Gear = 0
        m_mode_msg = ModeCmd(MorA=self.m_mode_MorA, EStop=self.m_mode_EStop, Gear=m_mode_Gear)

        rospy.logdebug(' MPS2KPH(linear_vel) %s', self.mps_to_kph(linear_vel))
        rospy.logdebug(' Linear Vel : %s Steer Angle: %s', linear_vel, steering_angle)

        m_drive_msg = DriveCmd(KPH=abs(linear_vel), Deg=math.degrees(steering_angle))

        m_drive_msg.KPH = min(self.MAX_KPH, m_drive_msg.KPH)
        m_drive_msg.Deg = min(self.MAX_DEGREE, m_drive_msg.Deg)
        m_drive_msg.Deg = max(-self.MAX_DEGREE, m_drive_msg.Deg)

        self.m_pub_mode.publish(m_mode_msg)
        self.m_pub_drive.publish(m_drive_msg)

    def ModeCallback(self, msg):
        self.m_mode_MorA = msg.MorA
        self.m_mode_EStop = msg.EStop

    def UpdateOdometry(self, current_time):
        current_time = rospy.Time.now()

        try:
            # Get the latest transform between 'odom' and 'base_link'
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Failed to lookup transform')
            return

        # Create a TransformStamped message from the latest transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform = transform.transform

        # Broadcast the TransformStamped message
        self.tf_broadcaster.sendTransform(odom_trans)

        # Create an Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set the position and orientation of the Odometry message,
        # based on the latest transform
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        odom.pose.pose.orientation = transform.transform.rotation

        # Set the velocity of the Odometry message (in the 'base_link' frame),
        # based on the data from the erp42_interface
        odom.twist.twist.linear.x = self.erp42_interface_.linear_vel
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.erp42_interface_.angular_vel

        # Publish the Odometry message
        self.odom_publisher.publish(odom)

        

    # def UpdateWheelState(self):

    #     steer_tf = geometry_msgs.msg.TransformStamped()
    #     odom = Odometry()
    #     steer_angle = quaternion_from_euler(0, 0, 10)

    #     steer_tf.header.stamp = rospy.Time.now()
    #     steer_tf.header.frame_id = 'base_link'
    #     steer_tf.child_frame_id = 'front_left_steer_link'



    #     steer_tf.transform.translation.x = self.erp42_interface_.odom_x
    #     steer_tf.transform.translation.y = self.erp42_interface_.odom_y
    #     steer_tf.transform.translation.z = 0.0
    #     steer_tf.transform.rotation.x = steer_angle[0]
    #     steer_tf.transform.rotation.y = steer_angle[1]
    #     steer_tf.transform.rotation.z = steer_angle[2]
    #     steer_tf.transform.rotation.w = steer_angle[3]
        
    #     self.m_odom_broadcaster.sendTransform(steer_tf)
    #     self.m_pub_odom.publish(odom)

    def Run(self):
        while not rospy.is_shutdown():
            self.m_current_time = rospy.Time.now()
            self.m_delta_time = self.m_current_time - self.m_last_time
            self.m_last_time = self.m_current_time

            diff_time = float(self.m_delta_time.secs) + float(self.m_delta_time.nsecs) * 1e-9

            # self.erp42_interface_.CalculateOdometry(diff_time)
            # self.UpdateOdometry(self.m_current_time)
            # self.UpdateWheelState()
            self.rate_.sleep()  # rate is 50hz
            rospy.sleep(0.05)

        rospy.spin()


if __name__ == "__main__":
    ed = ERP42Driver()

    ed.Run()