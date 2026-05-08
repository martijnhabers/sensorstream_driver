# sensorstream_driver

A ROS 2 driver that streams sensor data from an iPhone running the SensorStream app. Supports both WiFi and USB connections, publishing camera, depth, IMU, GPS, and point cloud data as standard ROS 2 messages.

## Published Topics

| Topic                    | Type                                | QoS                      | Description              |
|--------------------------|-------------------------------------|--------------------------|--------------------------|
| `color_image`            | `sensor_msgs/msg/Image`             | Best effort, last 5 | Raw BGR camera frames for RViz |
| `color_image/compressed` | `sensor_msgs/msg/CompressedImage`   | Best effort, last 5 | JPEG camera frames       |
| `depth_image`            | `sensor_msgs/msg/Image`             | Reliable, last 10   | 16-bit depth (16UC1)     |
| `imu/data`               | `sensor_msgs/msg/Imu`               | Reliable, last 10   | Accelerometer & gyroscope|
| `gps/fix`                | `sensor_msgs/msg/NavSatFix`         | Reliable, last 10   | GPS position             |
| `pointcloud`             | `sensor_msgs/msg/PointCloud2`       | Best effort, last 5 | 3D point cloud           |

## Parameters

| Parameter   | Default | Description                          |
|-------------|---------|--------------------------------------|
| `wifi_port` | `5678`  | TCP port for WiFi connections        |
| `usb_port`  | `2345`  | Port for USB (PeerTalk) connections  |

## Requirements

- ROS 2 (Humble or later)
- `libprotobuf-dev` and `protobuf-compiler`
- `libusbmuxd-dev` (libusbmuxd 2.0)

Install dependencies on Ubuntu/Debian:

```bash
sudo apt install libprotobuf-dev protobuf-compiler libusbmuxd-dev
```

## Building

```bash
# From your colcon workspace
colcon build --packages-select sensorstream_driver
source install/setup.bash
```

## Usage

### Basic

```bash
ros2 run sensorstream_driver sensorstream_node
```

In another terminal, camera frames can be checked with:

```bash
ros2 topic echo /color_image/compressed --qos-reliability best_effort --once --no-arr
```

### With custom ports

```bash
ros2 run sensorstream_driver sensorstream_node --ros-args \
  -p wifi_port:=9000 \
  -p usb_port:=3000
```

### Using multiple devices

You can override the node name and namespace to run multiple instances for use with multiple iPhones.

Example:

```bash
# Device 1
ros2 run sensorstream_driver sensorstream_node --ros-args \
  -r __ns:=/iphone_1 \
  -r __node:=sensorstream_node_1 \
  -p wifi_port:=5678 \
  -p usb_port:=2345

# Device 2 (in a different terminal)
ros2 run sensorstream_driver sensorstream_node --ros-args \
  -r __ns:=/iphone_2 \
  -r __node:=sensorstream_node_2 \
  -p wifi_port:=5679 \
  -p usb_port:=2346
```



