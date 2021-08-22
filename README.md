# MA-LOAM (Model Aware Lidar Odometry and Mapping)

Given a 3D model of the environment as a prior, incorporates it in SLAM calculations for better accuracy

## Progress

- [ ] Setup a simulation environment for virtual lidar sensors
- [ ] Implement a variation of cluster iterative closest point
    - [ ] Selection
    - [ ] Matching
    - [ ] Weighting
    - [ ] Rejection
    - [ ] Error metrics
    - [ ] Minimization
- [ ] Integrate it into LOAM framework

### General comments on work in progress

Blensor is not enough for our needs. It is hard to automate and also does not provides the true pose of the scan origin. I will try out gazebo. If it becomes too expensive to run, we can record the scan data into rosbags and play it back later on

## How to

### Clone from git

This package uses git large file storage (lfs) to store the environments CAD models. On Ubuntu, you can install it using,

```bash
apt install git-lfs
```

You might need elevated privileges depending on your setup. Afterwards, setup git lfs using,

```bash
git lfs install
```

Finally, clone this like any other standard git repository

### Building and running 

Clone this package in a catkin workspace and build the package using:

```bash
catkin build --this --no-deps --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

For more detail (or in case your workspace was not setup using `catkin-tools`), take a look [here](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/). When the build finishes, simply launch environments using the provided launch files. For example,

```bash
roslaunch ma_loam simple_corridor.launch            # Launches a light weight corridor model
roslaunch ma_loam skarpnack.launch -v --screen      # Launches the Skarpnack station model
```

The `-v` flag prints more verbose outputs whereas `--screen` redirects the outputs to screen. More info can be found [here](https://wiki.ros.org/roslaunch/Commandline%20Tools). 

<ins>**Note:**</ins> If gazebo complains that it is unable to set the model state, just ignore it. We retry later when gazebo loads the models
