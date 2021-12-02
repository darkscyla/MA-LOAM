#!/usr/bin/env python3

# --- Standard Imports ---
import math
import threading
from typing import List, Optional

# --- ROS Imports ---
import rospy
import tf
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateResponse


class ModelStateSetter:
    def __init__(self, name) -> None:
        # Setup the model state service
        topic = "/gazebo/set_model_state"
        rospy.wait_for_service(topic)
        self.set_state = rospy.ServiceProxy(topic, SetModelState)

        self.sensor_name = name
        self.global_frame = "world"

        # Setup the common model state
        self.state = ModelState()
        self.state.model_name = self.sensor_name
        self.state.reference_frame = self.global_frame

        self.last_pose = None
        self.cv = threading.Condition()
        self.available = False

        # Spawn subscriber to monitor model states
        self.state_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.state_cb, queue_size=1
        )
        self.pose_sub = None
        self.twist_sub = None

    def set_model_pose(self, pose) -> None:
        state = ModelState()

        # Set the model state
        state.model_name = self.state.model_name
        state.reference_frame = self.state.reference_frame

        state.pose.position.x = pose[0]
        state.pose.position.y = pose[1]
        state.pose.position.z = pose[2]

        state.pose.orientation.x = pose[3]
        state.pose.orientation.y = pose[4]
        state.pose.orientation.z = pose[5]
        state.pose.orientation.w = pose[6]

        # Wait until model with sensor name is available in gazebo
        if not self.available:
            with self.cv:
                self.cv.wait()

        response: SetModelStateResponse = self.set_state(state)
        rospy.loginfo(
            f"Pose set for: {self.sensor_name} with success: {response.success}"
        )

    def listen(self) -> None:
        self.pose_sub = rospy.Subscriber("/set_pose", Pose, self.pose_cb, queue_size=10)
        self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_cb, queue_size=1)

    def state_cb(self, states: ModelStates):
        if not self.available:
            if self.sensor_name in states.name:
                self.available = True

                with self.cv:
                    self.cv.notify()
                # Some grace time to make sure pose is set
                rospy.sleep(1)
        else:
            # Get index of sensor and save the pose of the sensor
            idx = states.name.index(self.sensor_name)
            self.last_pose: Pose = states.pose[idx]

    def pose_cb(self, pose: Pose) -> None:
        self.state.pose = pose
        self.set_state(self.state)

    def to_local(self, twist: Twist) -> Twist:
        """Converts the twist to local coordinates"""
        yaw = tf.transformations.euler_from_quaternion(
            [
                self.last_pose.orientation.x,
                self.last_pose.orientation.y,
                self.last_pose.orientation.z,
                self.last_pose.orientation.w,
            ]
        )[2]
        _cos = math.cos(yaw)
        _sin = math.sin(yaw)

        _twist = Twist()
        _twist.angular = twist.angular
        _twist.linear.x = twist.linear.x * _cos - twist.linear.y * _sin
        _twist.linear.y = twist.linear.x * _sin + twist.linear.y * _cos
        _twist.linear.z = twist.linear.z

        return _twist

    def vel_cb(self, twist: Twist) -> None:
        """Set the velocity of the model"""
        if not self.last_pose is None:
            # Set state and publish
            self.state.pose = self.last_pose
            self.state.twist = self.to_local(twist)
            self.set_state(self.state)


def parse_pose(pose: List[float]) -> Optional[List[float]]:
    if not isinstance(pose, list):
        return None

    # Case of x y z r p y, we need to convert rpy to quaternion
    if len(pose) == 6:
        return pose[:3] + list(
            tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
        )
    # Quaternion version is good to go
    if len(pose) == 7:
        return pose
    # Otherwise, the pose is ill-formed
    return None


def main() -> None:
    rospy.init_node("model_state_node")

    # Fetch sensor info
    sensor_name = rospy.get_param("sensor_name")
    raw_pose = rospy.get_param("sensor_pose_init", [])
    sensor_pose = parse_pose(raw_pose)

    model_pose = ModelStateSetter(sensor_name)

    if sensor_pose:
        model_pose.set_model_pose(sensor_pose)
    else:
        rospy.logerr(f"Invalid pose format: {raw_pose}")

    # Allows update through cmd vel topic
    model_pose.listen()
    rospy.spin()


if __name__ == "__main__":
    main()
