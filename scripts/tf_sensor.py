#!/usr/bin/env python3

# --- ROS Imports ---
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped
from gazebo_msgs.msg import LinkStates


class SensorTF:
    def __init__(self) -> None:
        """
        Publishes the tf of the sensor base_footprint
        """
        self.sensor_name = rospy.get_param("sensor_name", None)

    def run(self) -> None:
        # Compute the frame names
        self.bfp_frame = "base_footprint"
        self.bfp_link = f"{self.sensor_name}::{self.bfp_frame}"

        # Broadcasts tf frame between map and base_footprint
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.tf_rate = rospy.Rate(rospy.get_param("sensor_tf_rate", 10))

        # Subscribe to gazebo link state and forward the transform using ROS
        self.gazebo_link_sub = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self._cb, queue_size=1
        )

    def _cb(self, msg: LinkStates) -> None:
        if not self.bfp_link in msg.name:
            return

        idx: int = msg.name.index(self.bfp_link)
        bfp_pose: Pose = msg.pose[idx]

        # Create the transform
        ts = TransformStamped()

        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "world"
        ts.child_frame_id = self.bfp_frame

        ts.transform.translation = bfp_pose.position
        ts.transform.rotation = bfp_pose.orientation

        self.tf_br.sendTransform(ts)

        self.tf_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("sensor_tf_node")

    sensor_tf = SensorTF()

    if sensor_tf.sensor_name == None:
        rospy.logerr("Unable to fetch sensor name from the param server")
    else:
        sensor_tf.run()
        rospy.spin()
