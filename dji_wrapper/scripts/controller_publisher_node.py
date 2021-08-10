#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import turtlesim.msg

import math


class TransformPublisherController():

    def __init__(self):
        self.controller_br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/dji_osdk_ros/velocity',
                     geometry_msgs.msg.Vector3Stamped,
                     self.handle_velocity)

        rospy.Subscriber('/dji_osdk_ros/imu',
                     sensor_msgs.msg.Imu,
                     self.handle_imu)

        #self.timer = rospy.Timer(rospy.Duration(.1), self.publish_pose)

        self.position_mutex = True
        self.current_velocity_x = 0
        self.current_velocity_y = 0
        self.current_velocity_z = 0

        self.orientation_mutex = True
        self.current_orientation_x = 0
        self.current_orientation_y = 0 #do i want quaternions? no.
        self.current_orientation_z = 0

        print("Constructed!")






    def handle_velocity(self, msg):


        x_velocity = msg.vector.x
        y_velocity = msg.vector.y
        z_velocity = msg.vector.z

        while(not self.position_mutex):
            pass 

        self.position_mutex = False

        self.current_velocity_x = x_velocity
        self.current_velocity_y = y_velocity
        self.current_velocity_z = z_velocity

        self.position_mutex = True



    def handle_imu(self, msg):
        while(not self.orientation_mutex):
            pass 

        self.orientation_mutex = False

        self.current_orientation_x = msg.angular_velocity.x
        self.current_orientation_y = msg.angular_velocity.y
        self.current_orientation_z = msg.angular_velocity.z

        self.orientation_mutex = True



    def publish_pose(self):

        t1 = geometry_msgs.msg.TransformStamped()

        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = "map_parent"
        t1.child_frame_id = "joystick_1"
        print(self.current_velocity_z) 
        t1.transform.translation.x = ((-.5)*(self.current_orientation_z+2.618)/(2.618+2.618)+.25)
        t1.transform.translation.y = ((-.5)*(self.current_velocity_z+5)/(5+5)+.25)
        t1.transform.translation.z = 0

        t1.transform.rotation.x = 1
        t1.transform.rotation.y = 0
        t1.transform.rotation.z = 0
        t1.transform.rotation.w = 0

        self.controller_br.sendTransform(t1)

        t2 = geometry_msgs.msg.TransformStamped()

        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = "map_parent"
        t2.child_frame_id = "joystick_2"
        # print(self.current_velocity_x) between 2 and -2
        # print(self.current_velocity_y)
        t2.transform.translation.x = ((-.5)*(self.current_velocity_x+3)/(6)+.25)
        t2.transform.translation.y = ((-.5)*(self.current_velocity_y+3)/(6)+.25)
        t2.transform.translation.z = 0

        t2.transform.rotation.x = 1
        t2.transform.rotation.y = 0
        t2.transform.rotation.z = 0
        t2.transform.rotation.w = 0

        self.controller_br.sendTransform(t2)

        t3 = geometry_msgs.msg.TransformStamped()

        t3.header.stamp = rospy.Time.now()
        t3.header.frame_id = "map_parent"
        t3.child_frame_id = "controller"
        t3.transform.translation.x = 0
        t3.transform.translation.y = 0
        t3.transform.translation.z = 0

        t3.transform.rotation.x = 1
        t3.transform.rotation.y = 0
        t3.transform.rotation.z = 0
        t3.transform.rotation.w = 0

        self.controller_br.sendTransform(t3)        

        d1 = geometry_msgs.msg.TransformStamped()

        d1.header.stamp = rospy.Time.now()
        d1.header.frame_id = "map_parent"
        d1.child_frame_id = "dot_1"
        d1.transform.translation.x = 0
        d1.transform.translation.y = 0
        d1.transform.translation.z = 0

        d1.transform.rotation.x = 1
        d1.transform.rotation.y = 0
        d1.transform.rotation.z = 0
        d1.transform.rotation.w = 0

        self.controller_br.sendTransform(d1)          

        d2 = geometry_msgs.msg.TransformStamped()

        d2.header.stamp = rospy.Time.now()
        d2.header.frame_id = "map_parent"
        d2.child_frame_id = "dot_2"
        d2.transform.translation.x = 0
        d2.transform.translation.y = 0
        d2.transform.translation.z = 0

        d2.transform.rotation.x = 1
        d2.transform.rotation.y = 0
        d2.transform.rotation.z = 0
        d2.transform.rotation.w = 0

        self.controller_br.sendTransform(d2)  

if __name__ == '__main__':
    rospy.init_node('controller_node')

    t = TransformPublisherController()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t.publish_pose()
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            print("time went backward, but we carry on")
    
    print("exited")
