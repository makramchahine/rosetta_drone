import rospy
from std_msgs.msg import UInt8, Float32
from sensor_msgs.msg import BatteryState, NavSatFix, Imu, Joy
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, PointStamped
from dji_osdk_ros.msg import VOPosition

class Logger:


    def write_state(self):
        batt_msg = self.battery_state_msg
        battery_voltage = batt_msg.voltage
        battery_current = batt_msg.current
        battery_percentage = msg.percentage

        acc_msg = self.acc_ground_fused_msg

        acc_x = acc_msg.vector.x
        acc_y = acc_msg.vector.y
        acc_z = acc_msg.vector.z

        ang_vel_msg = self.angular_vel_msg
        ang_vel_x = ang_vel_msg.x
        ang_vel_y = ang_vel_msg.y
        ang_vel_z = ang_vel_msg.z

        attitude_msg = self.attitude_msg
        att_x = attitude_msg.quaterion.x
        att_y = attitude_msg.quaterion.y
        att_z = attitude_msg.quaterion.z
        att_w = attitude_msg.quaterion.w

        gps_health = self.gps_health_msg.data

        gps_msg = self.gps_position_msg
        lat = gps_msg.latitude
        lng = gps_msg.longitude
        gps_alt = gps_msg.altitude

        height_above_takeoff = self.height_above_takeoff_msg.data

        pos_msg = self.local_position_msg
        local_x = pos_msg.point.x
        local_y = pos_msg.point.y
        local_z = pos_msg.point.z

        rc_msg = self.rc_msg
        rc0 = rc_msg.axes[0]
        rc1 = rc_msg.axes[1]
        rc2 = rc_msg.axes[2]
        rc3 = rc_msg.axes[3]
        rc4 = rc_msg.axes[4]
        rc5 = rc_msg.axes[5]

        vel_msg = self.velocity_msg
        vx = vel_msg.vector.x
        vy = vel_msg.vector.y
        vz = vel_msg.vector.z

        #vo_pos_msg = self.vo_position_msg

        gimbal_msg = self.gimbal_msg
        gimbal_x = gimbal_msg.vector.x
        gimbal_y = gimbal_msg.vector.y
        gimbal_z = gimbal_msg.vector.z

        csv_row = self.fmt % (battery_voltage,battery_current,battery_percentage, acc_x,acc_y,acc_z,ang_vel_x,ang_vel_y,ang_vel_z,att_x,att_y,att_z,att_w,gps_health,lat,lng,gps_alt,height_above_takeoff,local_x,local_y,local_z,rc0,rc1,rc2,rc3,rc4,rc5,vx,vy,vz,gimbal_x,gimbal_y,gimbal_z)

        self.write_file.write(csv_row)




    def __init__(self):


        rospy.Subscriber('/dji_osdk_ros/battery_state', BatteryState, battery_state_cb)
        rospy.Subscriber('/dji_osdk_ros/acceleration_ground_fused', Vector3Stamped, acc_ground_fused_cb)
        rospy.Subscriber('/dji_osdk_ros/angular_velocity_fused', Vector3Stamped, angular_vel_fused_cb)
        rospy.Subscriber('/dji_osdk_ros/attitude', QuaternionStamped, attitude_cb)
        rospy.Subscriber('/dji_osdk_ros/gps_health', Uint8, gps_health_cb)
        rospy.Subscriber('/dji_osdk_ros/gps_position', NavSatFix, gps_position_cb)
        rospy.Subscriber('/dji_osdk_ros/height_above_takeoff', Float32, height_above_takeoff_cb)
        #rospy.Subscriber('/dji_osdk_ros/imu', Imu, imu_cb)
        rospy.Subscriber('/dji_osdk_ros/local_position', PointStamped, local_position_cb)
        rospy.Subscriber('/dji_osdk_ros/rc', Joy, rc_cb)
        rospy.Subscriber('/dji_osdk_ros/velocity', Vector3Samped, velocity_cb)
        rospy.Subscriber('/dji_osdk_ros/vo_position', VOPosition, vo_position_cb)
        rospy.Subscriber('/dji_osdk_ros/gimbal_angle', Vector3Stamped, gimbal_cb)

        self.battery_state_msg = None
        self.acc_ground_fused_msg = None
        self.angular_vel_msg = None
        self.attitude_msg = None
        self.gps_health_msg = None
        self.gps_position_msg = None
        self.height_above_takeoff_msg = None
        self.imu_msg = None
        self.local_position_msg = None
        self.rc_msg = None
        self.velocity_msg = None
        self.vo_position_msg = None
        self.gimbal_msg = None
        
        csv_fields = ['battery_voltage','battery_current','battery_percentage', 'acc_x','acc_y','acc_z','ang_vel_x','ang_vel_y','ang_vel_z','att_x','att_y','att_z','att_w','gps_health','lat','lng','gps_alt','height_above_takeoff','local_x','local_y','local_z','rc0','rc1','rc2','rc3','rc4','rc5','vx','vy','vz','gimbal_x','gimbal_y','gimbal_z']
        fmt_fields = ['%.3f' if s not in ['lat', 'lng'] else '%.7f'for s in csv_fields ]
        header = ','.join(csv_fields)
        self.fmt = ','.join(fmt_fields) + '\n'
        self.write_file = None

    def battery_state_cb(self, msg):
        self.battery_state_msg = msg
    def acc_ground_fused_cb(self, msg):
        self.acc_ground_fused_msg = msg
    def angular_vel_fused_cb(self, msg):
        self.angular_vel_msg = msg
    def attitude_cb(self, msg):
        self.attitude_msg = msg
    def gps_health_cb(self, msg):
        self.gps_health_msg = msg
    def gps_position_cb(self, msg):
        self.gps_position_msg = msg
    def height_above_takeoff_cb(self, msg):
        self.height_above_takeoff_msg = msg
    #def imu_cb(self, msg):
    #    self.imu_msg = msg
    def local_position_cb(self, msg):
        self.local_position_msg = msg
    def rc_cb(self, msg):
        self.rc_msg = msg
    def velocity_cb(self, msg):
        self.velocity_msg = msg
    def vo_position_cb(self, msg):
        self.vo_position_msg = msg
    def gimbal_cb(self, msg):
        self.gimbal_msg = msg
