#!/usr/bin/env python3

# --- Standard Imports ---
import argparse
import os
import pathlib
from sys import path

# --- Open3D Imports ---
import open3d as o3d

# --- ROS Imports ---
import rospkg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Converts environment stl models to point cloud and saves them as ply file"
    )
    parser.add_argument(
        "-n",
        "--name",
        type=str,
        help="Name of the environment",
        required=True,
    )
    parser.add_argument(
        "-s",
        "--samples",
        type=int,
        help="Number of sampling points",
        required=True,
    )
    parser.add_argument(
        "-p",
        "--postfix",
        type=str,
        help="Environment stl file postfix",
        nargs="?",
        default="",
    )

    return vars(parser.parse_args())


def main() -> None:
    args = parse_args()

    # Get the environment info
    rospack = rospkg.RosPack()
    pkg_base_dir = rospack.get_path("ma_loam")
    env_name = args["name"]

    env_base_dir = os.path.join(
        pkg_base_dir,
        f"resources/environments/{env_name}/",
    )
    env_stl_path = os.path.join(
        env_base_dir,
        f"{env_name}{args['postfix']}.stl",
    )

    if not pathlib.Path(env_stl_path).is_file():
        print(f"STL file not found: {env_stl_path}")
        return

    # Load the STL file and sample it
    mesh = o3d.io.read_triangle_mesh(env_stl_path)

    print(f"=> Sampling environment with {args['samples']}")
    pt_cloud = mesh.sample_points_poisson_disk(args["samples"])

    # Save the pointcloud
    env_pcd_path = os.path.join(
        env_base_dir,
        f"{env_name}{args['postfix']}.pcd",
    )
    print("=> Writing pcd file")
    o3d.io.write_point_cloud(env_pcd_path, pt_cloud)
    print("=> Done!")


if __name__ == "__main__":
    main()
