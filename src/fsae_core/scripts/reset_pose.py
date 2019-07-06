#!/usr/bin/env python
import rospy
from robot_localization.srv import SetPose

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseResetti():
    def __init__(self):
        rospy.wait_for_service('set_pose')
        # Thanks Andrew!
        self.dumbVar = rospy.Subscriber('platform/user_input', Int32, self.switchCallback)
        rospy.wait_for_service('set_pose')
        self.poseSetter = rospy.ServiceProxy('set_pose', SetPose)

    def switchCallback(self, switchVal):
        switchVal = switchVal.data
        # Switch up (not default position)
        if switchVal == 1811:
            rospy.logwarn('Resetting local odometry to 0')
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'odom'
            self.poseSetter(pose)


if __name__ == '__main__':
    rospy.init_node('PoseResetti')
    resetti = PoseResetti()
    rospy.spin()
