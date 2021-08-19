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
