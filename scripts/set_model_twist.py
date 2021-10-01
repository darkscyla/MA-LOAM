#!/usr/bin/env python3

# --- ROS Imports ---
import rospy
import tf
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateResponse

# --- Standard Imports ---
import argparse
from typing import Dict, List, Union


def ros_success(msg):
    rospy.loginfo("\x1b[6;30;42m" + msg + "\x1b[0m")


class ModelPose:
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

        # Retry every second to update the position of the model
        # Gazebo might take time to load everything
        rate = rospy.Rate(1)
        success = False

        while not success:
            response: SetModelStateResponse = self.set_state(state)
            success = response.success
            rate.sleep()

        ros_success(f"Pose successfully set for: {self.sensor_name}")

    def listen(self) -> None:
        rospy.sleep(1)

        self.state_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.state_cb, queue_size=1
        )
        self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_cb, queue_size=1)

    def state_cb(self, states: ModelStates):
        # Get index of sensor and save the pose of the sensor
        idx = states.name.index(self.sensor_name)
        self.last_pose: Pose = states.pose[idx]

    def vel_cb(self, twist: Twist) -> None:
        if not self.last_pose is None:
            # Set state and publish
            self.state.pose = self.last_pose
            self.state.twist = twist
            self.state
            self.set_state(self.state)


def parse_arguments() -> Dict[str, str]:
    parser = argparse.ArgumentParser(description="Sets the pose of the model")
    parser.add_argument(
        "-n",
        "--name",
        type=str,
        help="Name of the model whose pose should be set",
        required=True,
    )
    parser.add_argument(
        "-p",
        "--pose",
        nargs="+",
        type=float,
        help="Pose of the model with translation followed by orientation"
        ' --> "x y z r p y" or "x y z x y z w"',
        required=True,
    )
    args, _ = parser.parse_known_args()
    return vars(args)


def parse_pose(pose: str) -> Union[List[float], None]:
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


if __name__ == "__main__":
    # Fetch command line arguments
    args = parse_arguments()
    pose = parse_pose(args["pose"])

    rospy.init_node("model_state_node")

    if pose:
        model_pose = ModelPose(args["name"])
        model_pose.set_model_pose(pose)
    else:
        rospy.logerr(f"Invalid pose format: {args['pose']}")

    # Allows update through cmd vel topic
    model_pose.listen()
    rospy.spin()
