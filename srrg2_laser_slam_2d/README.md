# Package `srrg2_laser_slam_2d`

Multi-cue 2D-LiDAR-based SLAM pipeline. It can support multiple rangefinders and wheel odometry.

## How to build
The `srrg2_laser_slam_2d` is developed using our `srrg2` framework.
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.
Please follow this guide to build and run `srrg2_laser_slam_2d` on your machine:

1. initialize the `srrg2` Catkin workspace following the guide [here](https://github.com/srrg-sapienza/srrg2_core/tree/master/srrg2_core). As indicated in the aforementioned guide, we suggest to have a directory in which you clone all the `srrg2` repositories (referred here as `SRRG2_SOURCE_ROOT`) and a directory that contains the Catkin workspace (referred here as `SRRG2_WS_ROOT`)

2. clone all the `srrg2` dependencies of this package
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_cmake_modules.git # basic cmake-modules
git clone https://gitlab.com/srrg-software/srrg_hbst.git # VPR library (to compute loop closures in Visual-SLAM pipelines)
git clone https://github.com/srrg-sapienza/srrg2_core.git # core data-structures and
git clone https://github.com/srrg-sapienza/srrg2_solver.git # solver (both for registration and global optimization)
git clone https://github.com/srrg-sapienza/srrg2_qgl_viewport.git # viewport
git clone https://github.com/srrg-sapienza/srrg2_slam_interfaces.git # SLAM interfaces
```

3. clone this repository
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_laser_slam_2d.git
```

4. link all the required packages in your Catkin workspace
```bash
cd <SRRG2_WS_ROOT>/src
ln -s <SRRG2_SOURCE_ROOT>/srrg2_cmake_modules .
ln -s <SRRG2_SOURCE_ROOT>/srrg_hbst/ .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_core/srrg2_core .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_core/srrg2_core_ros .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_solver/srrg2_solver .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_qgl_viewport/srrg2_qgl_viewport .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_slam_interfaces/srrg2_slam_interfaces .
```

5. build using Catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_laser_slam_2d
```

6. [OPTIONAL] build unit-tests using catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_laser_slam_2d --catkin-make-args tests
```

## How to run
You should use our package [`srrg2_executor`](https://github.com/srrg-sapienza/srrg2_executor) to run the pipeline. In the [`configs`](https://github.com/srrg-sapienza/srrg2_laser_slam_2d/tree/master/configurations) directory you will find two configuration file templates that can be used with [this dataset](https://drive.google.com/open?id=1el30W7cLEKDpAOdjFCInjpRB738Rx9bb):

* [`stage_segway_double_config_LASER_0`](https://github.com/srrg-sapienza/srrg2_laser_slam_2d/tree/master/configurations/stage_segway_double_config_LASER_1.json): uses only the front rangefinder + wheel odometry.
* [`stage_segway_double_config_MULTI`](https://github.com/srrg-sapienza/srrg2_laser_slam_2d/tree/master/configurations/stage_segway_double_config_MULTI.json): uses both front and rear rangefinders + wheel odometry.

###### Example Output
![multi-lidar-output](../configurations/OUTPUT_MAP_stage_segway_double_config_MULTI.png)
