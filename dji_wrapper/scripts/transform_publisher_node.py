#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import turtlesim.msg

from tf.transformations import quaternion_from_euler

import math


class TransformPublisher():

    def __init__(self):
        self.drone_br = tf2_ros.TransformBroadcaster()
        self.gimbal_br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/dji_osdk_ros/gps_position',
                     sensor_msgs.msg.NavSatFix,
                     self.handle_gps_position)

        rospy.Subscriber('/dji_osdk_ros/attitude',
                     geometry_msgs.msg.QuaternionStamped,
                     self.handle_attitude)

        rospy.Subscriber('/dji_osdk_ros/gimbal_angle',
                     geometry_msgs.msg.Vector3Stamped,
                     self.handle_gimbal_angle)

        #self.timer = rospy.Timer(rospy.Duration(.1), self.publish_pose)

        self.position_mutex = True
        self.current_position_x = 0
        self.current_position_y = 0
        self.current_position_z = 0

        self.orientation_mutex = True
        self.current_orientation_w = 1
        self.current_orientation_x = 0
        self.current_orientation_y = 0
        self.current_orientation_z = 0

        self.gimbal_mutex = True
        self.gimbal_roll = 0
        self.gimbal_pitch = 0
        self.gimbal_yaw = 0

        self.origin_lat =  42.5215478236
        self.origin_lon = -71.6066135646

        self.origin_altitude = 109



        print("Constructed!")






    def handle_gps_position(self, msg):


        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude - self.origin_altitude


        dx = (self.origin_lon-lon)*40000*math.cos((self.origin_lat+lat/1000)*math.pi/360)/360*1000
        dy = (self.origin_lat-lat)*40000/360*1000


        while(not self.position_mutex):
            pass 

        self.position_mutex = False

        self.current_position_x = dx
        self.current_position_y = dy
        self.current_position_z = alt

        self.position_mutex = True

        #print("gps handled!")






    def handle_attitude(self, msg):
        while(not self.orientation_mutex):
            pass 

        self.orientation_mutex = False

        self.current_orientation_w = msg.quaternion.w
        self.current_orientation_x = msg.quaternion.x
        self.current_orientation_y = msg.quaternion.y
        self.current_orientation_z = msg.quaternion.z

        self.orientation_mutex = True

        #print("attitude handled!")


    def handle_gimbal_angle(self, msg):
        while(not self.gimbal_mutex):
            pass 

        self.gimbal_mutex = False

        self.gimbal_roll = msg.vector.x*math.pi/180
        self.gimbal_pitch = msg.vector.y*math.pi/180
        self.gimbal_yaw = msg.vector.z*math.pi/180

        self.gimbal_mutex = True

        #print("attitude handled!")




    def publish_pose(self):

        t1 = geometry_msgs.msg.TransformStamped()

        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = "world"
        t1.child_frame_id = "base_link"
        t1.transform.translation.x = self.current_position_x
        t1.transform.translation.y = self.current_position_y
        t1.transform.translation.z = self.current_position_z

        t1.transform.rotation.x = self.current_orientation_x
        t1.transform.rotation.y = self.current_orientation_y
        t1.transform.rotation.z = self.current_orientation_z
        t1.transform.rotation.w = self.current_orientation_w

        self.drone_br.sendTransform(t1)


        t2 = geometry_msgs.msg.TransformStamped()

        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "gimbal"
        t2.transform.translation.x = 0.24
        t2.transform.translation.y = 0
        t2.transform.translation.z = -.37

        #wants radians
        q = quaternion_from_euler(self.gimbal_roll, self.gimbal_pitch, self.gimbal_yaw)

        t2.transform.rotation.x = q[0]
        t2.transform.rotation.y = q[1]
        t2.transform.rotation.z = q[2]
        t2.transform.rotation.w = q[3]

        self.drone_br.sendTransform(t2)


if __name__ == '__main__':
    rospy.init_node('transform_publisher_node')

    t = TransformPublisher()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t.publish_pose()
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            print("time went backward, but we carry on")
    
    print("exited")

    #rospy.spin()