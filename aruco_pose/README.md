# Positioning with ArUco markers

ROS 2 (Jazzy) port of the `aruco_pose` package from [CopterExpress/clover](https://github.com/CopterExpress/clover/tree/master/aruco_pose) (ROS 1 Noetic).

`aruco_pose` package consists of two composable nodes: `aruco_detect` (`aruco_pose::ArucoDetect`) detects individual ArUco-markers and estimates their poses, `aruco_map` (`aruco_pose::ArucoMap`) detects maps of markers using `aruco_detect` output.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aruco_pose --symlink-install
source install/setup.bash
```

## Quick start

To run the markers and maps detector in a component container:

```bash
ros2 launch aruco_pose sample.launch.py
```

The launch file doesn't start a camera: run any camera driver that publishes `main_camera/image_raw` and `main_camera/camera_info`, or pass other topic names with the launch arguments `image_raw`, `camera_info`. Other arguments: `length` (markers' side length, default `0.33`), `map` (path to the map file, default `map/map.txt`).

For example, to feed the detector with a test image:

```bash
ros2 run image_publisher image_publisher_node aruco_pose/test/map.png --ros-args -r __ns:=/main_camera \
    -p frame_id:=main_camera_optical -p camera_info_url:=file://$PWD/aruco_pose/test/camera_info.yaml
```

## Differences from the ROS 1 version

* Nodelets are replaced with composable nodes (`rclcpp_components`); `nodelet_plugins.xml` is removed.
* `dynamic_reconfigure` is replaced with ordinary ROS 2 parameters (`ros2 param set`): `enabled`, `length` and the detector parameters of `aruco_detect`, `enabled`, `map` and `image_axis` of `aruco_map` can be changed at runtime.
* The parameter namespace separator is `.` instead of `/`: `length_override.<id>`, `markers.frame_id`, `markers.child_frame_id_prefix`.
* `~length_override` entries are read at start only (as `length_override.<id>` parameters); use the `aruco_detect/set_length_override` service to change lengths at runtime.
* Private topics and services are named `<node_name>/<topic>`, as before (e. g. `aruco_detect/markers`).
* Camera topics are subscribed with the sensor data QoS (best effort); `aruco_map/map`, `aruco_map/image`, `aruco_map/visualization` are published with the transient local QoS (the replacement of ROS 1 latched topics), and `map_markers` is subscribed with the same QoS.
* `aruco_pose/Marker` and `MarkerArray` messages are unchanged, but live in the `aruco_pose/msg` namespace; services: `aruco_pose/srv/SetMarkers`.
* `genmap.py` is started with `ros2 run aruco_pose genmap.py`; `-o` writes to the `map` directory of the installed package.
* Tests use `launch_testing` instead of `rostest`.

## aruco_detect node

`aruco_detect` detects ArUco markers on the image, publishes list of them (with poses), TF transformations, visualization markers and processed image for debugging.

It's recommended to run it in the same component container as the camera node to avoid copying images.

### Parameters

* `~dictionary` (*int*) – ArUco dictionary (default: 2)
  * 0 = DICT_4X4_50
  * 1 = DICT_4X4_100,
  * 2 = DICT_4X4_250,
  * 3 = DICT_4X4_1000,
  * 4 = DICT_5X5_50,
  * 5 = DICT_5X5_100,
  * 6 = DICT_5X5_250,
  * 7 = DICT_5X5_1000,
  * 8 = DICT_6X6_50,
  * 9 = DICT_6X6_100,
  * 10 = DICT_6X6_250,
  * 11 = DICT_6X6_1000,
  * 12 = DICT_7X7_50,
  * 13 = DICT_7X7_100,
  * 14 = DICT_7X7_250,
  * 15 = DICT_7X7_1000,
  * 16 = DICT_ARUCO_ORIGINAL
* `~estimate_poses` (*bool*) – estimate single markers' poses (default: true)
* `~send_tf` (*bool*) – send TF transforms (default: true)
* `~frame_id_prefix` (*string*) – prefix for TF transforms names, marker's ID is appended (default: `aruco_`)
* `~length` (*double*) – markers' sides length
* `~length_override.<id>` (*double*) – length of the marker with the specified id
* `~known_vertical` (*string*) – known vertical (Z axis) of all the markers as a frame
* `~flip_vertical` – flip vertical vector

### Topics

#### Subscribed

* `image_raw` (*sensor_msgs/Image*) – camera image
* `camera_info` (*sensor_msgs/CameraInfo*) – camera calibration info
* `map_markers` (*aruco_pose/MarkerArray*) – list of markers to disable TF transform publishing

#### Published

* `~markers` (*aruco_pose/MarkerArray*) – list of detected markers with their corners and poses
* `~visualization` (*visualization_msgs/MarkerArray*) – visualization markers for rviz
* `~debug` (*sensor_msgs/Image*) – debug image with detected markers

### Published transforms

* `<camera_frame>` => `<frame_id_prefix><id>` – markers' poses

## aruco_map node

`aruco_map` node estimates position of markers map.

### Parameters

* `~map` – path to text file with markers list
* `~frame_id` – published frame id (default: `aruco_map`)
* `~known_vertical` – known vertical (Z axis) of markers map as a frame
* `~flip_vertical` – flip vertical vector
* `~image_width` – debug image width (default: 2000)
* `~image_height` – debug image height (default: 2000)
* `~image_margin` – debug image margin (default: 200)
* `~image_axis` – whether debug image should contain axis (default: true)
* `~dictionary` (*int*) – ArUco dictionary (default: 2) - should be the same as `dictionary` parameter of `aruco_detect` node

Map file has one marker per line with the following line format:

```
marker_id marker_length x y z yaw pitch roll
```

Where yaw, pitch and roll are extrinsic rotation around Z, Y, X axis, respectively.

See examples in [`map`](map/) directory.

### Topics

#### Subscribed

* `image_raw` (*sensor_msgs/Image*) – camera image (used for debug image)
* `camera_info` (*sensor_msgs/CameraInfo*) – camera calibration info (used for debug image)
* `markers` (*aruco_pose/MarkerArray*) – list of markers detected by `aruco_detect` node

#### Published

* `~pose` (*geometry_msgs/PoseWithCovarianceStamped*) – estimated map pose
* `~map` (*aruco_pose/MarkerArray*) – list of markers in the loaded map
* `~image` (*sensor_msgs/Image*) – planarized map image
* `~visualization` (*visualization_msgs/MarkerArray*) – markers map visualization for rviz
* `~debug` (*sensor_msgs/Image*) – debug image with detected markers and map axis

### Published transforms

* `<camera_frame>` => `<map_name>` – markers map pose

## Running tests

Command for running tests:

```bash
colcon test --packages-select aruco_pose && colcon test-result --verbose
```

## Copyright

Copyright © 2018 Copter Express Technologies. Author: Oleg Kalachev.

Distributed under MIT License (https://opensource.org/licenses/MIT).

ROS 2 port based on the original [`aruco_pose`](https://github.com/CopterExpress/clover/tree/master/aruco_pose) package, Copyright (c) 2018 Copter Express Technologies, released under the MIT license (see [LICENSE](../LICENSE)).
