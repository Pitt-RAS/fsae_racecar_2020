#!/usr/bin/env python
from time import sleep

import rospy

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


if __name__ == '__main__':
    rospy.init_node('get_odom_point', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    sleep(5)
    now = rospy.Time.now()
    transform = tf_buffer.lookup_transform('odom', 'base_link', now, rospy.Duration(10))
    robot = PointStamped()
    robot.header.stamp = now
    robot.header.frame_id = 'base_link'
    robot.point.x = 0
    robot.point.y = 0

    robot = do_transform_point(robot, transform)
    print("[{}, {}]".format(robot.point.x, robot.point.y))
