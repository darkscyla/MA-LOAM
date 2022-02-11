# MA-LOAM (Model Aware Lidar Odometry and Mapping)

Given a 3D model of the environment as a prior, incorporates it in SLAM calculations for better accuracy

## How to clone from git

Just clone like any other repository. Initially, large files were on git LFS thus requiring special treatment but now, it has been cleaned.

## How to build

Clone this package in a catkin workspace, change directory to it and build the package using (given that you have all the dependencies installed):

```bash
catkin build --this --no-deps --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

For more detail (or in case your workspace was not setup using `catkin-tools`), take a look [here](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/).


## How to run
There are different ways to run MA-LOAM. Read instructions below for more detail. 

### With Gazebo

In cases where only the trajectory is saved in the bag file, we must run Gazebo for the LiDAR scans. To run the given model with gazebo, enter the following command:

```bash
roslaunch ma_loam haus_fzk.launch
```

This will launch the `haus_fzk` environment. If the environment name is `my_env`, it is expected that the model is under `resources/environments/my_env/my_env.stl`. Take a look at the `haus_fzk` environment directory for more information. It is also possible that the user want to use a different model for mesh features (for example, empty unfurnished enviroment). For such a case, the model is expected to be in `resources/environments/my_env/my_envTAG.stl`. The `TAG` can be specified in the launch file. For example, a cleaned environment for `haus_fzk` is stored with the name `haus_fzk_clean.stl`. Therefore the `TAG` is `_clean`. To run a simulation where gazebo and mesh correspondences use different model, enter the following:

```bash
roslaunch ma_loam haus_fzk.launch mesh_postfix:=_clean
```

More options can be found in the launch file.

### With bag files

Running with bag files is a bit more tricky as data can be under different topics. Consider a bag with the following structure:

```markdown
topics: /robot/lidar_3d/points     X msgs    : sensor_msgs/PointCloud2                
        /tf                        X msgs    : tf2_msgs/TFMessage      (N connections)
        /tf_static                 X msgs    : tf2_msgs/TFMessage                     
```

To run the bag file, the `PointCloud2` topic must be remapped to `/velodyne_points` topic.

```bash
rosbag play my_bag.bag /robot/lidar_3d/points:=/velodyne_points --topics /robot/lidar_3d/points /tf /tf_static
```

Additionally, the LiDAR frame is expected to be named `velodyne`. In case it is named differently, for example `diff_frame`, a simple static frame in the launch file can solve the issue.

```xml
    <node pkg="tf" type="static_transform_publisher" name="$(anon velodyne)" args="0 0 0 0 0 0 1 velodyne diff_frame 1" />
```

Finally, a frame between robot `base_footprint` and `velodyne` must be published. This can be set to identity as well. The reason this exist is because usually, the pose is set using the robot `base_footprint`. The implementation follows this convention. For a sample launch file, take a look at either `haus_fzk_bag.launch` or `empty_room.launch`.  

### With your model

Running your own models should be easy. Just take a look at provided examples and follow the naming convention for the environment files.

## Deep Global Registration

For MA-LOAM to work, a good initial pose estimate is necessary. The user should provide it in the launch file if known. Alternatively, [DGR](https://github.com/chrischoy/DeepGlobalRegistration) can be used for the initial pose estimate. Due to multiple extra requirements to run DGR, it is not integrated directly to the MA-LOAM pipeline. Instructions on how to run DGR can be found on their official website (requires a lot of packages like CUDA, pytorch complied for GPU, and Minkowski Engine to name a few). Once setup, download `ResUNetBN2C-feat32-kitti-v0.3.pth` from the official repository of DGR. The script named `scripts/dgr_mod/deep_global_pose_estimation.py` can then be used to estimate the initial pose. Remember to provide both the environment model and the scan as `pcd` files. The implementation was updated to support API changes in `open3d.0.13.0`. The official implementation may be used as well in case of older version of open3d.
