# `clover` ROS 2 package

A bundle for autonomous navigation and drone control: a port of the `clover` package of
[CopterExpress/clover](https://github.com/CopterExpress/clover) (ROS 1 Noetic) to **ROS 2 Jazzy**.

The package provides:

* `simple_offboard` – simplified control of the drone in OFFBOARD mode with ROS services (`navigate`, `get_telemetry`, `land`, ...);
* `vpe_publisher`, `optical_flow` – visual position estimation and optical flow for the flight controller;
* `led` – high level control of an LED strip and indication of the flight events;
* `rc`, `shell`, `camera_markers` – auxiliary nodes;
* `selfcheck` – automatic check of the drone configuration;
* launch files, configuration of `mavros` and usage examples.

> **The port is verified only by building and by automated tests with mock nodes. Nothing has been checked on a real
> drone or in a simulator.** See [what is not verified](#not-verified-on-hardware) and [NOTES.md](NOTES.md) for details.

## Installation

Requirements: Ubuntu 24.04, [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html).

Clone the repository to a colcon workspace. The `clover` package depends on `aruco_pose` and `led_msgs` packages of the same repository:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MaxX-Akela/clover-origin-ros2.git
```

Install the dependencies (including `mavros`) using `rosdep`:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy
```

`selfcheck` needs `pymavlink`, which is not available in `rosdep`:

```bash
pip install pymavlink
```

Build the packages (on memory constrained platforms use one worker and a few make jobs):

```bash
cd ~/ros2_ws
MAKEFLAGS=-j2 colcon build --packages-select aruco_pose led_msgs clover --parallel-workers 1
source install/setup.bash
```

To complete `mavros` installation, install `geographiclib` datasets (`mavros_node` doesn't start without them):

```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

You may optionally install udev rules to provide `/dev/px4fmu` symlink to your PX4-based flight controller connected over USB:

```bash
sudo cp ~/ros2_ws/src/clover-origin-ros2/clover/udev/99-px4fmu.rules /lib/udev/rules.d
```

### mavros version

`mavros` **2.16.0 or newer** is required for the full functionality. `mavros` 2.15.1 doesn't pass the parameters
given on start to its plugins, so:

* the package runs `mavros_params` node, which sets the parameters of the plugins from [`config/mavros.yaml`](config/mavros.yaml)
  using the parameter services, when `mavros` is started;
* **`set_attitude` service doesn't work with `mavros` 2.15.1**: `mavros/setpoint_attitude/attitude` topic can't be enabled,
  the service responds with success, but the setpoints are not passed to the flight controller.

The package has been built and tested with `mavros` 2.15.1 only. Operating with `mavros` 2.16.0 is not verified.

### Missing drivers

There are no ROS 2 drivers for the LED strip (`ws281x`) and for the rangefinder (`vl53l1x`) yet. Without them:

* **LED strip**: `led` node waits for `led/set_leds` service and `led/state` topic of the driver forever, `led/set_effect` service is not available.
* **Rangefinder**: nobody publishes to `rangefinder/range` topic, so the rangefinder data is not passed to the flight controller,
  and `selfcheck` always reports `no rangefinder data from Raspberry`.

The launch files print a warning for each package, that is not installed, and go on.

## Running

To start connection to the flight controller and all the nodes, use:

```bash
ros2 launch clover clover.launch.py
```

The package is configured to connect to `/dev/px4fmu` by default (see [installation](#installation)). Arguments of `clover.launch.py`:

| Argument | Default | Description |
|---|---|---|
| `fcu_conn` | `usb` | connection to the flight controller: `usb`, `uart`, `udp`, `sitl`, `hitl`, `none` |
| `fcu_ip` | `127.0.0.1` | IP address of the flight controller for the UDP connections |
| `fcu_sys_id` | `1` | MAVLink system ID of the flight controller |
| `gcs_bridge` | `tcp` | bridge for a ground control station: `tcp`, `udp`, `udp-b`, `udp-pb`, `false` |
| `gcs_host` | | host to bind for `udp-b` and `udp-pb` bridges |
| `web_video_server` | `true` | run `web_video_server` |
| `rosbridge` | `true` | run `rosbridge_server` and `tf2_web_republisher` |
| `main_camera` | `true` | run the main camera (`main_camera.launch.py`) |
| `optical_flow` | `true` | run `optical_flow` |
| `aruco` | `false` | run ArUco markers detection (`aruco.launch.py`) |
| `rangefinder_vl53l1x` | `true` | run the rangefinder driver (not available) |
| `led` | `true` | run the LED strip nodes (`led.launch.py`) |
| `rc` | `false` | run `rc` node |
| `force_init` | `true` | force the estimator to init by publishing zero pose |
| `simulator` | `false` | flag that we are operating on a simulated drone |

For example, to connect to PX4 SITL without the camera:

```bash
ros2 launch clover clover.launch.py fcu_conn:=sitl main_camera:=false
```

The other launch files (`mavros.launch.py`, `main_camera.launch.py`, `aruco.launch.py`, `led.launch.py`) may be run
separately; the camera orientation, the markers map, the LED strip, etc. are configured with their arguments
(`ros2 launch clover main_camera.launch.py --show-args`).

To check the configuration of the drone, run:

```bash
ros2 run clover selfcheck
```

## API usage

The autonomous flight API is the same as in ROS 1: the names of the services, their types, fields and constants are not changed
(see [the documentation of the original project](https://clover.coex.tech/en/simple_offboard.html)).

From the command line:

```bash
ros2 service call /get_telemetry clover/srv/GetTelemetry "{frame_id: map}"
ros2 service call /navigate clover/srv/Navigate "{x: 0.0, y: 0.0, z: 1.0, frame_id: body, auto_arm: true}"
ros2 service call /land std_srvs/srv/Trigger
```

From Python, using `service_proxy` function of `clover` package, which replaces `rospy.ServiceProxy`:

```python
import time
import rclpy
from clover import srv, service_proxy
from std_srvs.srv import Trigger

rclpy.init()

get_telemetry = service_proxy('get_telemetry', srv.GetTelemetry)
navigate = service_proxy('navigate', srv.Navigate)
land = service_proxy('land', Trigger)

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
time.sleep(5)

print('Perform landing')
land()
```

### `service_proxy`

`service_proxy(name, srv_type, timeout=None, wait_for_service=5.0)` returns a function, which calls the service synchronously
and returns the response.

* The request fields are passed as arguments (positional or keyword), the omitted fields have the default values.
  Integers are accepted for the float fields.
* `timeout` is the time to wait for the response in seconds, `None` is to wait forever. `TimeoutError` is raised on timeout.
* `wait_for_service` is the time to wait for the service to become available. `RuntimeError` is raised if the service is not available.
* The function may be called from any thread, including the callbacks of your own node: the proxies use a separate node,
  which is created on the first use and is spun in a background thread.
* `rclpy.init()` is called automatically, if it has not been called yet.

`long_callback` decorator for long-running topic callbacks (image processing) is available as in ROS 1: `from clover import long_callback`.

### Examples

The examples are in [`examples`](examples) directory (installed to `share/clover/examples`), run them with `python3`:

| Example | Description | Status |
|---|---|---|
| `get_telemetry.py` | print the state of the drone | run with a mock FCU in the tests |
| `navigate_wait.py` | take off, fly forward and land, waiting for the arrival | run with a mock FCU in the tests |
| `leds.py` | LED strip effects | run with a mock LED driver in the tests |
| `camera.py` | basic image processing | run with a mock camera (synthetic image) in the tests |
| `flight.py` | take off, fly forward and land | syntax check only |
| `flight_marker.py` | flight using ArUco markers | syntax check only |
| `gps.py` | flight using global coordinates | syntax check only |
| `subscriber.py` | subscribing to the rangefinder data | syntax check only |
| `red_circle.py` | following a red circle | syntax check only; needs `ros-jazzy-image-geometry` |

"Run in the tests" means that the example works with `simple_offboard` or `led` node and a mock of the flight controller
or of the driver. None of the examples has been run on a real drone or in a simulator.

## Tests

```bash
colcon build --packages-select clover --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select clover
colcon test-result --verbose
```

The tests (`launch_testing`) run the nodes with mock publishers and services instead of the flight controller, the camera
and the LED driver. `test/smoke_selfcheck.py` and `test/smoke_launch.py` are additional scripts to be run manually;
they need `mavros_node` (without a flight controller).

## Differences from ROS 1

The full list is in [NOTES.md](NOTES.md) (in Russian). The most important differences:

* **Launch**: `roslaunch clover clover.launch` → `ros2 launch clover clover.launch.py`. `blocks` argument is removed, `gcs_host` argument is added (replaces `ROS_HOSTNAME` for `udp-b` and `udp-pb`).
  `simulator.launch` is not ported.
* **Python API**: `rospy` → `rclpy`; `rospy.ServiceProxy` → `clover.service_proxy`; `rospy.sleep` → `time.sleep` in the examples.
  Float fields of the messages don't accept integers in `rclpy` (`service_proxy` converts them).
* **Parameters**: `/` in the names is replaced with `.` (`notify/low_battery/threshold` → `notify.low_battery.threshold`).
  There are no global parameters: the parameters of other nodes are read using their parameter services.
* **`simple_offboard`, `vpe_publisher`, `optical_flow`**: the local and FCU frames are read from the parameters of
  `mavros/local_position` node on start (waiting up to 5 s), or may be set with the new `local_frame` and `fcu_frame` parameters.
  Calls to `mavros` services have timeouts (`offboard_timeout`, `arming_timeout`, `land_timeout`).
* **`optical_flow`** is a component (`clover::OpticalFlow`) instead of a nodelet; `dynamic_reconfigure` is replaced with `enabled` parameter.
* **Camera**: `cv_camera` → `v4l2_camera`; the nodelet manager `main_camera_nodelet_manager` → the components container `main_camera_container`.
  There are no equivalents of `rate`, `capture_delay` and `rescale_camera_info` parameters of `cv_camera`
  (the calibration is rescaled by the launch file).
* **`mavros`**: `mavlink/to` and `mavlink/from` topics don't exist, `/uas1/mavlink_sink` and `/uas1/mavlink_source` are used instead
  (`mavlink_topic`, `mavlink_from_topic` parameters of `rc` and `selfcheck`); `mavros/param/get` service doesn't exist,
  PX4 parameters are read using `mavros/param/get_parameters`; there is no `visualization` node in `mavros_extras`.
* **`led`**: errors are read from `/rosout` instead of `/rosout_agg`; `set_leds` service of the driver is called asynchronously.
* **`selfcheck`**: run with `ros2 run clover selfcheck`; the nodes are searched in the ROS graph instead of the processes list;
  the checks of `ROS_HOSTNAME` are removed.
* **QoS**: subscriptions to the telemetry and the images are best effort; latched topics are transient local.
* **Web**: see [Web](#web). `clover_blocks`, `clover_description` and `clover_simulation` packages are not ported.

## Web

The web tools (`www`) are installed to `share/clover/www`. Requires `rosbridge_server`, `tf2_web_republisher`,
`web_video_server` and the `roswww_static` package of this repository.

1. `ros2 launch clover clover.launch.py` starts `rosbridge_websocket` (port 9090), `tf2_web_republisher` and `web_video_server` (port 8080)
   (`rosbridge`, `web_video_server` arguments).
2. `ros2 run roswww_static update` links `share/clover/www` to `~/.ros/www/clover`.
3. Serve `~/.ros/www` with a static web server (nginx or another, following symlinks): `http://<host>/clover/`.

Differences from ROS 1:

* `tf2_web_republisher` for ROS 2 has only the action, no `republish_tfs` service, so `viz.js` sends the goal directly
  through the rosbridge protocol instead of using `ROSLIB.TFClient` (the bundled `roslib.js` is not changed).
* Message types are in the ROS 2 form (`sensor_msgs/msg/Image`), time fields are `sec`/`nanosec`, the ROS distribution is fixed to `jazzy`.
* The drone model in `viz.html` (`/vehicle_marker`) is not shown: there is no `visualization` node in `mavros_extras` for Jazzy.
* The links to `clover_blocks` and the web terminal (Butterfly) in `index.html` do not work.

The pages have not been opened in a browser in the development environment, see [Not verified on hardware](#not-verified-on-hardware).

## Not verified on hardware

Everything listed here has been checked only with mock nodes or has not been run at all:

* flights and any interaction with a real or simulated flight controller: OFFBOARD mode, arming, landing, setpoints,
  `navigate_global`, the MAVLink shell in `selfcheck`, time synchronization diagnostics, `fcu_conn` options except `udp` and `none`;
* `mavros` 2.16.0 and `set_attitude` with it;
* the camera: `v4l2_camera` is not installed in the development environment, its parameters are taken from the documentation;
  `image_proc` (rectification) and `topic_tools` (throttled image topic, rangefinder relay) have not been run;
* optical flow and ArUco markers with real images (checked with synthetic images only);
* the web pages in a browser (`index`, `gcs`, `topics`, `viz`, `console`, `aruco_map`): only the rosbridge protocol side is tested
  (`test/test_web.py`), see [Web](#web); `web_video_server` has not been run;
* the LED strip and the rangefinder (no drivers);
* running on Raspberry Pi and on the Clover image: `clover.service`, network, boot duration and `vcgencmd` checks of `selfcheck`;
* `flight.py`, `flight_marker.py`, `gps.py`, `subscriber.py`, `red_circle.py` examples;
* the CI workflow (`.github/workflows/ci.yml`) has not been run.

## License

MIT License, Copyright (c) 2018 Copter Express Technologies. See [LICENSE](../LICENSE).
The original project: [CopterExpress/clover](https://github.com/CopterExpress/clover).
