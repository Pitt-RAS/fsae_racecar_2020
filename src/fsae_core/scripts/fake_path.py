#!/usr/bin/env python
import math
import rospy
import numpy as np

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, PoseStamped


class FakePathNode(object):

    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._raw_path = rospy.get_param('~path')
        self._path = []
        self._step_size = rospy.get_param('~step_size')
        self._frame_id = rospy.get_param('~path_frame')
        self._odom_frame_id = rospy.get_param('~odom_frame')
        self._publisher = rospy.Publisher('/path', Path, queue_size=5)

    def transform_points(self):
        now = rospy.Time.now()
        try:
            transform = self._tf_buffer.lookup_transform(self._odom_frame_id, self._frame_id, now, rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logwarn("fake_path: Couldn't lookup transform {} -> {}".format(self._frame_id, self._odom_frame_id))
            return
        self._path = []
        header = Header(frame_id=self._frame_id, stamp=now)
        for point in self._raw_path:
            waypoint = PointStamped()
            waypoint.header = header
            waypoint.point.x = point[0]
            waypoint.point.y = point[1]

            waypoint = do_transform_point(waypoint, transform)
            self._path.append([waypoint.point.x, waypoint.point.y])

    def publish_path(self):
        if len(self._path) < 2:
            rospy.logwarn("fake_path: Path is empty")
            return
        path = list(self._path)
        path_msg = Path()
        path_msg.header.frame_id = self._odom_frame_id

        last = np.array(path.pop(0))

        while len(path) != 0:
            curr = np.array(path.pop(0))

            num_steps = math.ceil(np.linalg.norm(last-curr)/self._step_size)

            plan = zip(np.linspace(last[0], curr[0], num_steps), np.linspace(last[1], curr[1], num_steps))

            for point in plan:
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = point[0]
                pose_msg.pose.position.y = point[1]
                path_msg.poses.append(pose_msg)

            last = curr
        self._publisher.publish(path_msg)


if __name__ == '__main__':
    rospy.init_node('fake_path')

    node = FakePathNode()
    rate = rospy.Rate(1)
    retransform_period = rospy.Duration(5)
    last_retransform = rospy.Time(0)

    while not rospy.is_shutdown():
        if rospy.Time.now() - last_retransform >= retransform_period:
            rospy.loginfo("fake_path: Path retransform")
            node.transform_points()
            last_retransform = rospy.Time.now()
        node.publish_path()
        rate.sleep()
