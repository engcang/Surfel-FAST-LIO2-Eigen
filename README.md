<h1 align="center"><span>Surfel FAST-LIO2 Eigen</span></h1>

+ This repository provides an Eigen-only FAST-LIO2 state estimator with a two-level hierarchical voxel Surfel map.
+ MTK, `MTK_BUILD_MANIFOLD`, IKFoM, ikd-Tree, and OpenMP are not used.
+ A ROS-independent C++ core is shared by separate ROS1 and ROS2 wrappers.

<p align="center">
  <img src="assets/benchmark_overview.svg" alt="FAST-LIO2 Original and Surfel FAST-LIO2 Eigen benchmark comparison" width="100%" />
</p>

<br>

## Highlights
+ Explicit fixed-size Eigen implementation of the 17-dimensional FAST-LIO2 error state, including the SO(3) perturbation and two-dimensional gravity error.
+ Eigen-only iterated error-state Kalman filter, IMU propagation, point undistortion, measurement iteration, and covariance update.
+ Hierarchical point-to-plane Surfel map with dense and oneTBB concurrent-hash backends selectable once at startup.
+ PCL centroid-based `VoxelGrid` scan downsampling, preserving the estimator input used during accuracy validation.
+ Initial-IMU gravity alignment published as `map_frame → odometry_frame` without changing the estimator state or map.
+ Livox, Velodyne, Ouster, and MARSIM point-cloud preprocessing.

<br>

## Dependencies
+ `ROS1 Noetic` or `ROS2 Jazzy`
+ `C++` >= 17
+ `Eigen3`
+ `PCL` >= 1.8
+ `oneTBB`
+ `livox_ros_driver` for ROS1 Livox messages or `livox_ros_driver2` for ROS2 Livox messages

<br>

## How to install
+ Install the ROS2 package with

```bash
cd ~/<your_ros2_workspace>/src
git clone https://github.com/engcang/surfel-fast-lio2-eigen.git

cd ~/<your_ros2_workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select surfel_fast_lio2_eigen \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

+ Install the ROS1 package with

```bash
cd ~/<your_catkin_workspace>/src
git clone https://github.com/engcang/surfel-fast-lio2-eigen.git

cd ~/<your_catkin_workspace>
source /opt/ros/noetic/setup.bash
catkin build surfel_fast_lio2_eigen --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

<br>

## How to use
+ Run the ROS2 node with

```bash
ros2 launch surfel_fast_lio2_eigen run.launch.py \
    config_file:=$(ros2 pkg prefix surfel_fast_lio2_eigen)/share/surfel_fast_lio2_eigen/config/avia.yaml \
    rviz:=false
```

+ Run the ROS1 node with

```bash
roslaunch surfel_fast_lio2_eigen run.launch \
    config_file:=$(rospack find surfel_fast_lio2_eigen)/config/avia.yaml \
    rviz:=false
```

+ Select the Surfel map backend in the config YAML before startup.

```yaml
surfel:
  use_concurrent_hash_map: false
```

+ `false` selects the dense backend and is recommended for CPU-constrained onboard computers.
+ `true` selects the oneTBB concurrent-hash backend when lower mapping latency is worth additional CPU use.
+ Configure the gravity-aligned parent frame and estimator output frame with `common.map_frame` and `common.odometry_frame`.

<br>

## Performance
+ The table summarizes the 12-sequence, three-run comparison represented in the figure above.

| Metric | FAST-LIO2 Original | Surfel FAST-LIO2 Eigen | Change |
| --- | ---: | ---: | ---: |
| Mean internal pipeline time | 15.514 ms/scan | 3.826 ms/scan | **4.05× faster**, 75.3% lower |
| Whole-process CPU | 56.219% of one core | 26.730% of one core | **52.5% lower** |
| Successful-run translation APE RMSE | 10.867 m | 0.250 m | **97.7% lower** |
| Successful runs | 34/36 | 34/36 | Equal under the common criteria |

+ The test set contains three Newer College, three 2021 HILTI, and six NTU VIRAL sequences, with each sequence executed three times at one-times bag playback.
+ APE is translation error after EVO rigid SE(3) alignment without scale correction. The reported APE is the macro mean over successful runs; failures are not silently included as valid accuracy measurements.
+ FAST-LIO2 Original ran with ROS1 Noetic in the prepared container, while the Surfel implementation ran with ROS2 Jazzy on the host. The inputs, playback rate, and CPU affinity were held constant, but middleware and dependency versions differed.
+ These measurements describe the tested machine and configurations rather than a universal speed or accuracy guarantee. The compact source values and plotting command are in [`benchmark/`](benchmark/README.md).

<br>

### ● Troubleshooting
+ Start the node before bag playback and allow ROS2 DDS discovery to finish; dropping the first LiDAR or IMU messages can change initialization.
+ The LiDAR-to-IMU translation and rotation parameters require exactly 3 and 9 values, respectively.
+ Use the sensor-specific YAML as a starting point. Avia keeps a larger local map because of its longer measurement range.
+ The HILTI `uzh_tracking_area_run2` recording used in validation requires a `0.3 s` input start offset because its initial segment produces an unsuitable first Surfel map.

<br>

## Layout
+ `surfel_fast_lio2_eigen/include/` contains the ROS-independent preprocessing, state, Lie math, ESIKF, IMU propagation, and Surfel map implementations.
+ `ros1/` contains the ROS1 message adapter, application, configs, launch file, and node entry point.
+ `ros2/` contains the ROS2 message adapter, application, configs, launch file, and node entry point.
+ `third_party/` contains vendored header-only dependencies with their original notices.
+ `benchmark/` contains the compact benchmark summary and plot generator.

<br>

## Acknowledgements
+ This repository is a derivative of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO). The repository as a whole is therefore distributed under GNU GPL version 2; source-file notices inherited from FAST-LIO2 and LOAM are retained.
+ The hierarchical voxel Surfel mapping design follows [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry) and its associated paper.
+ `ankerl::unordered_dense` is vendored under the MIT License. Eigen, PCL, oneTBB, ROS, and Livox driver components remain under their respective licenses.

<br>

## LICENSE
+ This repository is free software distributed under the [GNU General Public License, version 2 only](LICENSE).
+ You may use, study, modify, and redistribute it under the GPL-2.0-only terms. Distributed modifications must preserve applicable copyright and license notices and remain GPL-compatible.
