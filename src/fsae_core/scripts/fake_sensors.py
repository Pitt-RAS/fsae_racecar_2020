#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from magellan_core.msg import (ObstacleStamped, ObstacleStampedArray)
from visualization_msgs.msg import Marker, MarkerArray


class FakeObstacles(object):
    def __init__(self, obstacles):
        self._marker_pub = rospy.Publisher('/obstacle_markers',
                                           MarkerArray,
                                           queue_size=5)

        self._obst_pub = rospy.Publisher('/obstacles',
                                         ObstacleStampedArray,
                                         queue_size=5)

        self._obstacle_msg = ObstacleStampedArray()
        self._marker_message = MarkerArray()

        marker_id = 1

        for name, values in obstacles.iteritems():
            # generate obstacle message
            _obst = ObstacleStamped()
            _obst.obst.x = values['x']
            _obst.obst.y = values['y']
            _obst.obst.width = values['width']
            _obst.obst.length = values['length']
            _obst.header.stamp = rospy.Time.now()
            _obst.header.frame_id = 'map'

            self._obstacle_msg.obstacles.append(_obst)

            # generate marker message
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.id = marker_id
            marker.ns = 'obstacles'

            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            marker.scale.x = _obst.obst.width
            marker.scale.y = _obst.obst.length
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration(0.1)

            center = Point()
            center.x = _obst.obst.x
            center.y = _obst.obst.y
            marker.points.append(center)

            self._marker_message.markers.append(marker)

            marker_id = marker_id + 1

    def _publish_obstacles(self):
        self._obst_pub.publish(self._obstacle_msg)

    def _publish_markers(self):
        self._marker_pub.publish(self._marker_message)

    def update(self):
        self._publish_obstacles()
        self._publish_markers()


if __name__ == '__main__':
    rospy.init_node('fake_sensors')

    # 20 hz update rate
    _rate = rospy.Rate(20)

    obsts = FakeObstacles(rospy.get_param('~fake_obstacles'))

    while not rospy.is_shutdown():
        obsts.update()
        _rate.sleep()
