# Lidar Obstacle Detection


## 1. Project Overview

<img src="images/ObstacleDetectionFPS.gif" width="700" height="400" />

This project is about processing raw lidar data with filtering, segmentation and clustering to detect other vehicles on the road. It covers the following key concepts:

- Lidar point cloud processing in real time
- Detecting Bounding boxes(objects) across frames in the video
- 3D RANSAC algorithm for ground plane segmentation
- KD-Tree data structure for Euclidean distance clustering




## 2. Dependencies for Running Locally

1. pcl >= 1.10 : refer to the [official instructions](https://pointclouds.org/downloads/)
    - linux : $ `sudo apt install libpcl-dev`
    - windows : PS> `.\vcpkg.exe install pcl[visualization]:x64-windows`
    - mac : $ `brew install pcl`

2. cmake >= 2.8

3. make >= 4.1 (Linux, Mac), 3.81 (Windows)

4. gcc/g++ >= 5.4

5. c++ >= c++11





## 3. Basic Build Instructions


1. Clone this repo.

2. Make a build directory in the top level project directory: `mkdir build && cd build`

3. Compile: `cmake .. && make`

4. Run it: `./environment`.