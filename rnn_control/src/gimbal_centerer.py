import dji_osdk_ros.srv as dji_srv
import rospy
from rospy.timer import TimerEvent


class GimbalCenterer:
    """
    Class that repeatedly invokes the gimbal_task_control rosservice every center_every seconds to center the drone
    gimbal over the course of center_duration seconds. To use, repeatedly call the center_
    """

    def __init__(self, center_every: float = 1, center_duration: float = 0.5):
        rospy.init_node("gimbal_centerer_node")
        self.center_duration = center_duration
        self.gimbal_task_service = rospy.ServiceProxy("gimbal_task_control", dji_srv.GimbalAction)

        rospy.Timer(rospy.Duration(center_every), self.center_gimbal)
        rospy.spin()

    def center_gimbal(self, event: TimerEvent):
        gimbal_action = dji_srv.GimbalAction
        gimbal_action.is_reset = False
        gimbal_action.payload_index = 1  # see common_types.h in dji sdk
        gimbal_action.rotationMode = 1  # absolute positioning
        gimbal_action.pitch = 0
        gimbal_action.roll = 0
        gimbal_action.yaw = 0
        gimbal_action.time = self.center_duration
        self.gimbal_task_service.call(gimbal_action)


if __name__ == "__main__":
    GimbalCenterer()
