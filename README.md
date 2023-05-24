ROS 2 driver for LitraTech's latest generation of mechanical 2D LiDARs running LDCP (**L**iDAR **D**ata and **C**ontrol **P**rotocol). Supported models are:
* **LTME-02A**
* R Series: **LT-R1, LT-R2**
* I Series: **LT-I1, LT-I2**

# Build and Install

## Dependencies

* C++11 capable compiler: **[Required]**
* OpenSSL: *[Optional]* The firmware updater will not be built if OpenSSL development files are missing; other parts of the package are not affected.

## Build the Package

Extract package source to your ROS 2 workspace's `src` directory, then build the workspace:

```
cd ~/ros2_ws/src
unzip ltme_node.zip
cd .. && colcon build
```

Or if you only want the package itself to be built:

```
cd ~/ros2_ws
colcon build --packages-up-to ltme_node
```

# Nodes

## ltme_node

Reads and publishes measurement data (ranges & intensities) from connected device. It also exposes several services for other nodes to query information about the device and control its operation mode.

### Published Topics

`scan` ([sensor_msgs/msg/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)): Laser scan measurements from connected device. The topic name can be changed with `remappings` parameter to `Node` constructor in Python launch file.

### Services

`~query_serial` ([std_srvs/srv/Empty](https://docs.ros2.org/foxy/api/std_srvs/srv/Empty.html)): Returns connected device's serial number as a [std_msgs/msg/String](https://docs.ros2.org/galactic/api/std_msgs/msg/String.html).

`~query_firmware_version` ([std_srvs/srv/Empty](https://docs.ros2.org/foxy/api/std_srvs/srv/Empty.html)): Returns connected device's firmware version as a [std_msgs/msg/String](https://docs.ros2.org/galactic/api/std_msgs/msg/String.html).

`~request_hibernation` ([std_srvs/srv/Empty](https://docs.ros2.org/foxy/api/std_srvs/srv/Empty.html)): Put device into standby mode. During standby the device will turn off its motor and laser to prevent wearing and save power; no data will be published until it's brought out of standby.

`~request_wake_up` ([std_srvs/srv/Empty](https://docs.ros2.org/foxy/api/std_srvs/srv/Empty.html)): Exit standby mode and resume normal operation.

### Parameters

`~device_model` (string) **[Required]**: Model name of the target device. Supported models are:
* `LTME-02A`
* R Series: `LT-R1`, `LT-R2`
* I Series: `LT-I1`, `LT-I2`

`~device_address` (string) **[Required]**: IP address of the target device. For supported models, factory default address is `192.168.10.160`.

`~frame_id` (string, default: "laser") *[Optional]*: Frame ID of published `LaserScan` messages.

`~invert_frame` (bool, default: "false") *[Optional]*: If this option is enabled, published `LaserScan` messages will have their X and Z axes inverted. When the device is mounted upside down, you can use this option to revert the inversion caused by mounting, and make it looks like the scans are from a device installed in a regular way.

`~angle_min` and `~angle_max` (float, default: -2.356 and 2.356 for models with 270-degree FoV; -3.142 and 3.142 for models with 360-degree FoV) *[Optional]*: Start and end angle of published laser scans (in radians). Default values for these parameters make the entire angular FoV available without clipping.

`~angle_excluded_min` and `~angle_excluded_max` (float, default: -3.142 and -3.142) *[Optional]*: Range of angle (in radians) for which data should be excluded from published laser scans. Default values for these parameters make the entire angular FoV available without clipping.

`~range_min` and `~range_max` (float, default: 0.05 and model-specific default max) *[Optional]*: Minimum and maximum range values that are considered valid for published laser scans. Range values out of these bounds should be ignored.
