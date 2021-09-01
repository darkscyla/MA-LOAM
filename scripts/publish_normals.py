#!/usr/bin/env python3

# --- ROS Imports ---
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


if __name__ == "__main__":
    rospy.init_node("publish_normal_node")

    pub = rospy.Publisher("normals", Marker, queue_size=1)
    rate = rospy.Rate(1)

    msg = Marker()

    msg.header.frame_id = "map"
    msg.header.stamp = rospy.get_rostime()
    msg.ns = "cloud_normals"
    msg.id = 0
    msg.type = Marker.ARROW
    msg.action = Marker.ADD

    msg.pose.orientation.w = 1
    msg.scale.x = 0.1
    msg.scale.y = 0.2
    # msg.scale.z = 1

    msg.color.r = 1
    msg.color.g = 0
    msg.color.b = 0
    msg.color.a = 1

    msg.points.append(Point(x=0, y=0, z=1))
    msg.points.append(Point(x=1, y=1, z=2))

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
