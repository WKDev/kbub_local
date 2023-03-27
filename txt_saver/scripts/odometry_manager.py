# 230313 kbub-chson
# odometry manager
# takes the state of erp42 as input, returns its odometry and tf state.
import rospy
import math
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import random as rd
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import time
from sensor_msgs.msg import JointState

class OdometryManager:
    def __init__(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.steer_angle = 0.0
        self.delta_encoder = 0
        self.encoder = 0
        self.last_encoder = 0
        self.wheel_pos = 0.0
        self.ns_ = 'erp42_driver'

        rospy.init_node('erp42_odometry_calculator')

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

        self.odom_pub = rospy.Publisher('/erp42_test/odom',Odometry,queue_size=10)
        self.serial_sub = rospy.Subscriber('erp42_serial', SerialFeedBack, self.CalculateOdometry)

        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.t1 = time.time()
        self.ResetOdometry()




    def CalculateOdometry(self, data):
        

        if type(data)!=float:
            delta_time = time.time()-self.t1
            self.t1 = time.time()

            rospy.loginfo('calculating odometry....')

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
            self.angular_vel = math.tan(np.deg2rad(data.steer)) * self.linear_vel / self.wheel_base

            self.odom_yaw += self.angular_vel
                    
            self.odom_x += self.delta_pos * math.cos(-np.deg2rad(self.odom_yaw))
            self.odom_y += self.delta_pos * math.sin(-np.deg2rad(self.odom_yaw))
            rospy.loginfo("Previous Encoder: %d", self.last_encoder)
            rospy.loginfo("Delta Pose: %f", self.delta_pos)
            rospy.loginfo("Linear Vel: %f", self.linear_vel)
            rospy.loginfo("Angular Vel: %f", self.angular_vel)
            rospy.loginfo("Odom X: %f", self.odom_x)
            rospy.loginfo("Odom Y: %f", self.odom_y)
            rospy.loginfo("Odom Yaw: %f", self.odom_yaw)

            odom_quat = quaternion_from_euler(0, 0, -np.deg2rad(self.odom_yaw))


            if math.isnan(self.delta_pos):
                self.delta_pos = 0.0
            if math.isnan(self.angular_vel):
                self.angular_vel = 0.0

            # self.odom.child_frame_id='base_link'
            # self.odom.pose.pose.position.x = 


                    # Create an Odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            # Set the position and orientation of the Odometry message,
            # based on the latest transform
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]
            
            # Set the velocity of the Odometry message (in the 'base_link' frame),
            # based on the data from the erp42_interface
            odom.twist.twist.linear.x = self.linear_vel
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.angular_vel

            # Publish the Odometry messagess
            self.odom_pub.publish(odom)


            ## publishing tf
            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            # q = tf_conversions.transformations.quaternion_from_euler(0, 0, )
            t.transform.rotation.x = odom_quat[0]
            t.transform.rotation.y = odom_quat[1]
            t.transform.rotation.z = odom_quat[2]
            t.transform.rotation.w = odom_quat[3]

            br.sendTransform(t)


            ## publishing tf
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['front_right_steer_joint','front_right_wheel_joint', 'front_left_steer_joint','front_left_wheel_joint','front_steer_joint','rear_right_wheel_joint', 'rear_left_wheel_joint','rear_wheel_joint']
            # rear_wheel_state = self.odom_x % (2*3.141592627)
            rear_wheel_state = data.encoder

            rad_steer = -np.deg2rad(data.steer)
            joint_state_msg.position= [rad_steer,rad_steer,rad_steer,rad_steer,rad_steer,rear_wheel_state,rear_wheel_state,rear_wheel_state]
            # joint_state_msg.velocity= [0,0,0,0,0,self.linear_vel,self.linear_vel,self.linear_vel]

            self.joint_state_pub.publish(joint_state_msg)


    def ResetOdometry(self):
        self.SetOdometry(0.0, 0.0, 0.0,0.0,0.0,0.0)
        self.odom_pub.publish(self.odom)

    def SetOdometry(self, x, y, z, r,p,yaw):
        self.odom.pose.pose.position.x = x
        self.odom.pose.pose.position.y = y
        self.odom.pose.pose.position.z = z
        q =quaternion_from_euler(r,p,yaw)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
            
if __name__ == "__main__":
    ei = OdometryManager()
    t1 = rospy.get_time()
    while not rospy.is_shutdown():
        time_diff = rospy.get_time()-t1

        t1 = rospy.get_time()
        rospy.sleep(0.02)
