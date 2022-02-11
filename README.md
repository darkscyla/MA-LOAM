# MA-LOAM (Model Aware Lidar Odometry and Mapping)

Given a 3D model of the environment as a prior, incorporates it in SLAM calculations for better accuracy

## How to

### Clone from git

Just clone like any other repository. Initially, large files were on git LFS thus requiring special treatment but now, it has been cleaned.

### Build and run 

Clone this package in a catkin workspace, change directory to it and build the package using (given that you have all the dependencies installed):

```bash
catkin build --this --no-deps --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

For more detail (or in case your workspace was not setup using `catkin-tools`), take a look [here](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/). When the build finishes, simply launch environments using the provided launch files. For example, run any of the following:

```bash
roslaunch ma_loam simple_corridor.launch            # Launches a light weight corridor model
roslaunch ma_loam skarpnack.launch -v --screen      # Launches the Skarpnack station model
```

The `-v` flag prints more verbose outputs whereas `--screen` redirects the outputs to screen. More info can be found [here](https://wiki.ros.org/roslaunch/Commandline%20Tools) 
