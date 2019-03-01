# Estimation-of-Quadrotor-State

**Problem Statement**

In phase 1, the pose of the drone was computed. In phase 2, the velocity was estimated. In this phase, Extended Kalman Filter (EKF) with additional sensor data (Inertial Measurement Unit or IMU) has been used to get better estimates of state.

**Running the Code**

To run the code, just run Wrapper.m script.

GTSAM Toolbox and the Quaternion Toolbox has been used. These have been added using a command in the wrapper function. Also, the data is read from inside the Wrapper function.

Machine Vision Toolbox for MATLAB of Peter Corke has been used to compute the homography. Computer Vision Toolbox of MATLAB have also used for Velocity Estimation.

