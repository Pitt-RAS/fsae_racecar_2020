#!/usr/bin/env python
from math import pi, sin, cos

import rospy
from std_msgs.msg import Float64, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point

import numpy as np

rospy.init_node("viz_turning_radius")


class TurningRadiusVisualization:
    def __init__(self):
        self._marker_pub = rospy.Publisher("/markers/turning_radius", Marker, queue_size=3)
        self._turning_radius_sub = rospy.Subscriber("/platform/cmd_turning_radius", Float64, self.update_turning_radius)

        self._marker = Marker()
        self._marker.header.frame_id = 'base_link'
        self._marker.type = Marker.LINE_STRIP
        self._marker.action = Marker.MODIFY
        self._marker.scale = Vector3(0.01, 1, 1)
        self._marker.color = ColorRGBA(0, 1, 0, 1)

        self._radius = 0

    def _get_points_on_radius(self):
        if self._radius == 0:
            start = Point()
            start.x = 0
            start.y = 0
            start.z = 0

            stop = Point()
            stop.x = 1
            stop.y = 0
            stop.z = 0
            return [start, stop]
        points = []
        invert = np.sign(self._radius)
        radius = np.abs(self._radius)
        for angle in np.linspace(pi/2, 0):
            point = Point()
            point.x = cos(angle) * radius
            point.y = (sin(angle) * radius - radius) * invert

            points.append(point)
            if point.x > 1 or np.abs(point.y > 1):
                break
        return points

    def update_turning_radius(self, turning_radius):
        self._radius = turning_radius.data

    def update(self):
        self._marker.points = self._get_points_on_radius()
        self._marker_pub.publish(self._marker)


rate = rospy.Rate(5)
visualization = TurningRadiusVisualization()
if __name__ == "__main__":
    while not rospy.is_shutdown():
        visualization.update()
        rate.sleep()
