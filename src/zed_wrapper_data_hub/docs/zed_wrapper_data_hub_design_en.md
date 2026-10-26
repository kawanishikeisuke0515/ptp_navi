# ZED Wrapper Data Hub Design Specification

Version: 0.2 draft
Date: 2026-09-15
Status: draft (based on the launch file and configuration in this repository; not all outputs have been verified on hardware)

## 1. Overview and Scope

`zed_wrapper_data_hub` is a standalone ROS 2 package that starts the official `zed_wrapper` with a dedicated configuration and exposes ZED camera images, sensor data, localization estimates, and status information as ROS 2 topics. This specification defines the package structure, dependencies, launch arguments, publication settings, topic fields, and standalone verification procedures.

The default target camera is the ZED2i. The official wrapper handles acquisition, estimation, and publication. This package defines no custom data processing nodes or message types.

## 2. Package Structure and Dependencies

```text
zed_wrapper_data_hub/
├── package.xml
├── setup.py / setup.cfg
├── resource/zed_wrapper_data_hub
├── zed_wrapper_data_hub/__init__.py
├── launch/zed_data_hub.launch.py
├── config/zed2i_data_hub.yaml
└── docs/
    ├── zed_wrapper_data_hub_design_ja.md
    ├── zed_wrapper_data_hub_design_ja.html
    ├── zed_wrapper_data_hub_design_en.md
    └── zed_wrapper_data_hub_design_en.html
```

The build type is `ament_python`. Runtime dependencies are `launch`, `launch_ros`, `zed_wrapper`, and `zed_description`. The runtime environment must also provide the ZED SDK, drivers, and a supported camera required by the official wrapper. Launch files, configuration files, and documentation are installed in the package's share directory.

### 2.1 Environment Setup and Source Checkout

Install ROS 2, the ZED SDK, and compatible CUDA and drivers first. SDK and ROS distribution support varies by wrapper version; consult the [official installation instructions](https://github.com/stereolabs/zed-ros2-wrapper#installation) and [releases](https://github.com/stereolabs/zed-ros2-wrapper/releases). This specification does not yet pin a combination verified on hardware.

The following procedure places both source repositories in the `ptp_navi` workspace. Replace `/path/to/ptp_navi` with the actual path and `humble` with your ROS distribution. Skip cloning any repository already present.

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ptp_navi

git clone https://github.com/stereolabs/zed-ros2-wrapper.git src/zed-ros2-wrapper
git clone https://github.com/stereolabs/zed-ros2-interfaces.git src/zed-ros2-interfaces
```

The resulting layout is:

```text
ptp_navi/src/
├── zed_wrapper_data_hub/
├── zed-ros2-wrapper/
└── zed-ros2-interfaces/
```

`zed-ros2-interfaces` provides `zed_msgs`. The official instructions also allow binary installation in some environments; this procedure builds both repositories from source. Do not place duplicate copies of the `zed_msgs` package in different source directories.

A fresh clone uses the default branch at the time of cloning. Before building, select a wrapper tag or commit compatible with your SDK and a corresponding interfaces version. Replace the following placeholders with actual values.

```bash
git -C src/zed-ros2-wrapper checkout <wrapper-tag-or-commit>
git -C src/zed-ros2-interfaces checkout <interfaces-tag-or-commit>
```

### 2.2 Dependency Installation and Build

Install `rosdep` and `colcon` first if they are not already available.

```bash
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions
# Run only if rosdep has not been initialized
sudo rosdep init
```

Run the following from the workspace root in a terminal with the ROS 2 environment sourced.

```bash
cd /path/to/ptp_navi
rosdep update
rosdep install --from-paths src/zed_wrapper_data_hub src/zed-ros2-wrapper src/zed-ros2-interfaces --ignore-src -r -y
colcon build --symlink-install --packages-up-to zed_wrapper_data_hub --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--packages-up-to` builds the Hub and its dependencies present in the workspace. Cloning the repositories does not install the ZED SDK or CUDA. Resolve dependency or build errors before launching.

```bash
ros2 pkg prefix zed_wrapper
ros2 pkg prefix zed_description
ros2 pkg prefix zed_msgs
ros2 pkg prefix zed_wrapper_data_hub
ros2 launch zed_wrapper_data_hub zed_data_hub.launch.py
```

Source ROS 2 and `install/setup.bash` in other terminals as well. For reproducibility, record the verified ROS distribution, SDK version, and the following commit IDs.

```bash
git -C src/zed-ros2-wrapper rev-parse HEAD
git -C src/zed-ros2-interfaces rev-parse HEAD
```

## 3. Launch Structure and Responsibilities

```text
zed_data_hub.launch.py
  └── zed_wrapper / zed_camera.launch.py
        ├── Open the ZED camera and grab frames
        ├── Publish images, sensors, localization, and status
        └── Provide the camera model and TF according to launch arguments
```

This package includes the official launch file and passes the configuration YAML through `ros_params_override_path`. The default is `config/zed2i_data_hub.yaml`. Topic creation, message field population, and actual publication conditions depend on the installed wrapper.

## 4. Launch Commands and Arguments

Build the package and source ROS 2 and the workspace environment before launching.

```bash
ros2 launch zed_wrapper_data_hub zed_data_hub.launch.py
```

To use a different configuration:

```bash
ros2 launch zed_wrapper_data_hub zed_data_hub.launch.py ros_params_override_path:=/absolute/path/to/custom.yaml
```

| Argument | Type | Default | Description |
| --- | --- | --- | --- |
| `camera_model` | string | `zed2i` | Camera model |
| `camera_name` | string | `zed2i` | Camera name and default namespace |
| `namespace` | string | Empty string | Namespace passed to the wrapper |
| `node_name` | string | `zed_node` | Wrapper node name |
| `serial_number` | string | `0` | Serial number; 0 selects wrapper default behavior |
| `camera_id` | string | `-1` | Device ID; -1 selects wrapper default behavior |
| `publish_urdf` | bool | `true` | Start the camera URDF and robot_state_publisher |
| `publish_tf` | bool | `true` | TF from odom to the camera |
| `publish_map_tf` | bool | `true` | TF from map to odom |
| `publish_imu_tf` | bool | `false` | Wrapper IMU TF publication |
| `enable_ipc` | bool | `true` | Intra-process communication |
| `use_sim_time` | bool | `false` | Use simulation time |
| `sim_mode` | bool | `false` | Simulation mode |
| `ros_params_override_path` | string | Package file `config/zed2i_data_hub.yaml` | Parameter override YAML |

## 5. Current Default Configuration

This section corresponds to `config/zed2i_data_hub.yaml` and the launch defaults. Using a different YAML file or overriding launch arguments changes these settings. Enabled publication means enabled in configuration; it does not guarantee successful reception or the configured rate on hardware.

### 5.1 Enabled Data and Features

| Data or feature | Setting |
| --- | --- |
| RGB images | `video.publish_rgb: true` |
| IMU | `sensors.publish_imu: true` |
| Magnetic field | `sensors.publish_mag: true` |
| Localization and odom / pose | `pos_tracking_enabled: true`, `publish_odom_pose: true` |
| Status information | `general.publish_status: true` |
| TF | `publish_tf: true`, `publish_map_tf: true` |
| Camera model | Launch argument `publish_urdf: true` |

### 5.2 Disabled Outputs and Features

| Data or feature | Settings (all false) |
| --- | --- |
| Left/right, raw, grayscale, and stereo images | `video.publish_left_right`, `publish_raw`, `publish_gray`, `publish_stereo` |
| Raw IMU, camera/IMU transform data, pressure, and temperature | `sensors.publish_imu_raw`, `publish_cam_imu_transf`, `publish_baro`, `publish_temp` |
| Depth images, depth information, point clouds, depth confidence, and disparity | `depth.publish_depth_map`, `publish_depth_info`, `publish_point_cloud`, `publish_depth_confidence`, `publish_disparity` |
| Area memory, 3D landmarks, separate pose-with-covariance output, and camera path | `pos_tracking.area_memory`, `publish_3d_landmarks`, `publish_pose_cov`, `publish_cam_path` |
| Mapping and detected planes | `mapping.mapping_enabled`, `publish_det_plane` |
| Object detection | `object_detection.od_enabled` |
| Body tracking | `body_tracking.bt_enabled` |
| Streaming server | `stream_server.stream_enabled` |
| Wrapper IMU TF | Launch argument `publish_imu_tf` |

`publish_pose_cov: false` disables the separate pose-with-covariance output. It does not remove the `covariance` fields from the Odometry message structure.

### 5.3 Acquisition and Estimation Settings

| Item | Current value |
| --- | --- |
| Camera name and model | `zed2i` |
| Acquisition resolution and frame rate | `HD720`, `grab_frame_rate: 30` |
| Processing FPS cap | `grab_compute_capping_fps: 30.0` |
| Publication resolution | `pub_resolution: CUSTOM`, `pub_downscale_factor: 2.0` |
| Image publication FPS | `pub_frame_rate: 15.0` |
| Sensor publication rate | `sensors_pub_rate: 50.0` |
| Depth computation | `depth_mode: NEURAL_LIGHT` |
| Point cloud rate setting | `point_cloud_freq: 5.0` (point cloud publication itself is OFF) |
| Tracking mode and IMU fusion | `pos_tracking_mode: AUTO`, `imu_fusion: true` |
| Estimation dimensions | `two_d_mode: true`, `fixed_z_value: 0.0` |
| Reference frames | `map_frame: map`, `odometry_frame: odom` |

Depth outputs are disabled, but depth computation remains enabled with `NEURAL_LIGHT` to preserve positional tracking. The configuration comments note that `NONE` may disable positional tracking in the official wrapper. Localization is currently configured for 2D operation, not unrestricted 3D motion output.

## 6. Topics and Fields

### 6.1 Recorded Topic Verification

The existing hardware verification record (2026-09-01) lists the following topics with `camera_name:=zed2i` and `node_name:=zed_node`. Names depend on the namespace and wrapper version. The two CameraInfo paths are candidates in that record; check their publishers and types in the actual environment.

| Topic | Data |
| --- | --- |
| `/zed2i/zed_node/rgb/color/rect/image` | Rectified color image |
| `/zed2i/zed_node/rgb/color/rect/camera_info` | Camera calibration information |
| `/zed2i/zed_node/rgb/color/rect/image/camera_info` | Alternative candidate path for camera calibration information |
| `/zed2i/zed_node/imu/data` | IMU orientation, angular velocity, and acceleration |
| `/zed2i/zed_node/imu/mag` | Magnetic field |
| `/zed2i/zed_node/odom` | Estimated position, orientation, and velocity |
| `/zed2i/zed_node/pose` | Estimated position and orientation |
| `/zed2i/zed_node/pose/status` | Positional tracking status |
| `/zed2i/zed_node/status/health` | Camera health |
| `/zed2i/zed_node/status/heartbeat` | Liveness notification |
| `/zed2i/joint_states` | Camera model joint states |
| `/zed2i/zed2i_description` | Camera URDF description |

### 6.2 Topic Types and Fields

The `/zed2i/zed_node` prefix is omitted below. This section describes standard message structures and upstream `zed_msgs` definitions. Section 6.1 records topic names verified on hardware; it does not establish that every field below has been verified. Use the commands in Section 6.6 to check actual types. For wrapper-specific types and status codes, the installed version's definitions take precedence.

| Topic | Expected message type | Fields and meaning |
| --- | --- | --- |
| `/rgb/color/rect/image` | `sensor_msgs/msg/Image` | `header`, `height` / `width` (pixels), `encoding` (pixel format), `is_bigendian`, `step` (bytes per row), `data` (pixel bytes) |
| `/rgb/color/rect/camera_info` or `/rgb/color/rect/image/camera_info` | `sensor_msgs/msg/CameraInfo` | `header`, `height` / `width`, `distortion_model`, `d` (distortion coefficients), `k` (3×3 intrinsic matrix), `r` (3×3 rectification rotation matrix), `p` (3×4 projection matrix), `binning_x/y`, `roi` (region of interest) |
| `/imu/data` | `sensor_msgs/msg/Imu` | `header`, `orientation`, `angular_velocity`, `linear_acceleration`, and covariance for each measurement; see Section 6.4 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | `header`, `magnetic_field.x/y/z` (magnetic field in T), `magnetic_field_covariance` (3×3, 9 elements) |
| `/odom` | `nav_msgs/msg/Odometry` | `header`, `child_frame_id`, `pose.pose`, `pose.covariance`, `twist.twist`, `twist.covariance`; see Section 6.3 |
| `/pose` | `geometry_msgs/msg/PoseStamped` | `header`, `pose.position.x/y/z` (m), `pose.orientation.x/y/z/w` (quaternion); no velocity or covariance |
| `/pose/status` | `zed_msgs/msg/PosTrackStatus` (verify on hardware) | `odometry_status` (VIO status), `spatial_memory_status` (tracking status within the map), `status` (deprecated in the referenced upstream definition) |
| `/status/health` | `zed_msgs/msg/HealthStatusStamped` (verify on hardware) | `header`, `serial_number`, `camera_name`, `low_image_quality`, `low_lighting`, `low_depth_reliability`, `low_motion_sensors_reliability` (problem flags), `scene_illuminance` (units of 0.1 lux) |
| `/status/heartbeat` | `zed_msgs/msg/Heartbeat` (verify on hardware) | `beat_count` (counter), `node_ns`, `node_name`, `full_name`, `camera_sn`, `svo_mode`, `simul_mode` |

`header` contains `stamp.sec` / `stamp.nanosec` (timestamp) and `frame_id` (reference frame name). Image optical frames use X right, Y down, and Z forward; distinguish these from robot body axes. Check `encoding` for channel order. Match image and CameraInfo resolutions and frames, and use `p` for projection into rectified images.

Interpret status codes using the constants for each field. The same numeric value can mean different things in different fields. Receiving a heartbeat alone does not establish valid localization.

Reference definitions: [Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg), [CameraInfo](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg), [MagneticField](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/MagneticField.msg), [PosTrackStatus](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/PosTrackStatus.msg), [HealthStatusStamped](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/HealthStatusStamped.msg), [Heartbeat](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Heartbeat.msg). These links point to upstream branches and do not pin the version installed on hardware.

### 6.3 Odometry Structure and Velocity Access

```text
nav_msgs/msg/Odometry
├── header
│   ├── stamp.sec / stamp.nanosec       # Data timestamp
│   └── frame_id                       # Reference frame for position/orientation
├── child_frame_id                     # Estimated child frame; velocity frame
├── pose                               # PoseWithCovariance
│   ├── pose                           # Pose: position and orientation
│   │   ├── position.x/y/z             # Position [m]
│   │   └── orientation.x/y/z/w        # Quaternion
│   └── covariance                     # 6×6, 36 elements
└── twist                              # TwistWithCovariance
    ├── twist                          # Twist: velocity itself
    │   ├── linear.x/y/z               # Linear velocity [m/s]
    │   └── angular.x/y/z              # Angular velocity [rad/s]
    └── covariance                     # 6×6, 36 elements
```

In `twist.twist`, the outer field contains velocity and covariance, while the inner field contains velocity itself. This accesses two nested fields with the same name; it does not perform a calculation twice. `pose.pose` follows the same structure.

```python
# msg is a received Odometry message
vx = msg.twist.twist.linear.x       # Linear velocity along X [m/s]
wz = msg.twist.twist.angular.z      # Angular velocity about Z [rad/s]
x = msg.pose.pose.position.x       # X position in the reference frame [m]
velocity_cov = msg.twist.covariance
```

Position and orientation are expressed in `header.frame_id`; velocity is expressed in `child_frame_id`. The current odometry frame setting is `odom`, but verify actual frame names in received messages. Before treating `linear.x` as robot forward speed, verify how the target frame's X axis relates to the robot's forward direction. Quaternion `z` is not the yaw angle; conversion requires the four components.

Covariance is a row-major 6×6 matrix. Pose order is position x/y/z followed by rotation about X/Y/Z; twist order is linear velocity x/y/z followed by angular velocity x/y/z. Diagonal indices are `0, 7, 14, 21, 28, 35`. Population of these values depends on the publisher; an all-zero array alone must not be interpreted as zero error.

Odometry contains no acceleration. Estimated velocity may fluctuate around zero at rest, so stationary detection should use thresholds and durations based on measurements. Distinguish a stale last value after updates stop from small fluctuations in fresh data. The presence of velocity fields does not establish their accuracy or computation method; compare against actual displacement and elapsed time.

Reference for frames and nesting: [Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg).

### 6.4 Reading IMU, Magnetic Field, and Pose

| IMU field | Meaning and units |
| --- | --- |
| `header.stamp` / `header.frame_id` | Measurement timestamp and sensor frame |
| `orientation.x/y/z/w` | Orientation quaternion (dimensionless) |
| `angular_velocity.x/y/z` | Angular velocity about each axis (rad/s) |
| `linear_acceleration.x/y/z` | Acceleration along each axis (m/s²) |
| `orientation_covariance` | Orientation covariance (3×3, 9 elements) |
| `angular_velocity_covariance` | Angular velocity covariance (3×3, 9 elements) |
| `linear_acceleration_covariance` | Acceleration covariance (3×3, 9 elements) |

Each IMU covariance matrix is row-major. All zeros mean unknown covariance; a first element of `-1` means the corresponding measurement is unavailable. Before using acceleration as robot motion acceleration, verify sensor axes, mounting orientation, and how the wrapper handles gravity in its output. Definition: [Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg).

`magnetic_field.x/y/z` is expressed along the axes of `header.frame_id`, in Tesla. Convert to µT using `value × 1e6`. All zeros in `magnetic_field_covariance` mean unknown covariance. This topic does not contain a heading angle itself.

`/pose` directly contains `pose.position` and `pose.orientation`, rather than Odometry's `pose.pose`. Current settings use `map` as the map frame, with `two_d_mode: true` and `fixed_z_value: 0.0`. Check `header.frame_id` for the actual reference frame.

### 6.5 TF and Camera Model Topics

Full topic names are shown below. URDF-related data is output by the overall launched system, including components such as robot_state_publisher.

| Topic | Expected message type | Main fields |
| --- | --- | --- |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Each entry in `transforms[]` contains `header` (parent frame and timestamp), `child_frame_id`, `transform.translation.x/y/z` (m), and `transform.rotation.x/y/z/w` (quaternion) |
| `/zed2i/joint_states` | `sensor_msgs/msg/JointState` | `header`, `name[]`, `position[]` (rad or m), `velocity[]` (rad/s or m/s), `effort[]` (N·m or N); arrays may be empty when data is unavailable |
| `/zed2i/zed2i_description` | `std_msgs/msg/String` | `data` contains the URDF XML string |

The current launch enables `publish_tf`, `publish_map_tf`, and `publish_urdf`. Conceptually, TF connects `map → odom → camera_link` and the camera's internal fixed frames. Actual camera frame names may include the camera name; verify them in received data.

### 6.6 Inspecting Types, Fields, and Values on Hardware

Run these commands in a ROS 2 environment with the Hub running.

```bash
# List published topics and types
ros2 topic list -t

# Check the type and show all fields of that type
ros2 topic type /zed2i/zed_node/odom
ros2 interface show nav_msgs/msg/Odometry

# Inspect frames, timestamp, pose, and velocity in one message
ros2 topic echo /zed2i/zed_node/odom --once

# Continuously display only velocity
ros2 topic echo /zed2i/zed_node/odom --field twist.twist

# Position and acceleration (use separate terminals as needed)
ros2 topic echo /zed2i/zed_node/odom --field pose.pose.position
ros2 topic echo /zed2i/zed_node/imu/data --field linear_acceleration

# Reception rate
ros2 topic hz /zed2i/zed_node/odom

# First obtain the actual wrapper-specific type name
ros2 topic type /zed2i/zed_node/pose/status
# If the output above is zed_msgs/msg/PosTrackStatus
ros2 interface show zed_msgs/msg/PosTrackStatus
```

Stop continuous output with Ctrl+C. `--field` only filters what is displayed; it does not change the original message structure. Distinguish configured rates from measured rates, and inspect `header.stamp` when investigating stopped updates.

## 7. Standalone Hub Verification

| Item | Check |
| --- | --- |
| Startup | The official wrapper opens the camera without startup errors |
| Active configuration | Inspect loaded settings with `ros2 param dump /zed2i/zed_node` (adjust for a different node name) |
| Types | Compare actual types with Section 6 using `ros2 topic list -t` and `ros2 interface show` |
| Images | Verify reception, actual resolution, encoding, and correspondence with CameraInfo |
| Sensors | Check IMU and magnetic field timestamp updates, axes, units, and missing data |
| Position and velocity | Record odom / pose, timestamps, and tracking status while moving and stationary; compare with actual motion |
| TF | Check parent/child frame names and transforms; verify that the same transform is not published redundantly |
| Rates | Measure reception rates with `ros2 topic hz` and record them separately from configured values |
| Disabled outputs | Verify that disabled outputs such as depth and point clouds are not being published |

## 8. Troubleshooting and Unverified Items

| Symptom | What to inspect |
| --- | --- |
| Camera cannot be opened | Connection, device selection, SDK environment, camera use by another process, wrapper logs |
| Topics cannot be received | Node startup, namespace, publication settings, ROS communication environment, publisher/subscriber QoS |
| Values stop updating | Continued message reception, `header.stamp`, tracking status, wrapper logs |
| Unexpected position or velocity | Coordinate frames, tracking status, stationary fluctuations, timestamps, comparison with actual motion |
| Status fields differ from the tables | Installed `zed_msgs` version and output of `ros2 interface show` |

This repository does not include the official wrapper implementation. Distinguish verification of Hub settings from verification of hardware outputs. Check and record the installed wrapper and SDK versions, actual topic types, how velocity and covariance fields are populated, gravity handling in IMU acceleration, and measured publication rates on the hardware environment.
