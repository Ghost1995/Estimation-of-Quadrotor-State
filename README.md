# Estimation-of-Quadrotor-State

**Problem Statement**

The task has been divided into two "modes", first being the SLAM mode and second, the Localization mode.

In the SLAM mode, a map is constructed (using GTSAM) of the world given April-Tag inputs by optimizing the factor graph. The algorithm simultaneously localizes itself in the map.

In Localization mode, this constructed map is used and, based on observations obtained, localizes itself in the map by estimating the 6DOF pose for each AprilTag visible to it in each frame using iSAM2 part of GTSAM package.

**Running the Code**

Simply run Wrapper.m script.

Machine Vision Toolbox for MATLAB of Peter Corke has been used to compute the homography.
