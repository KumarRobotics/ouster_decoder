# ouster_decoder

This decoder is intended as an alternative the [ouster-ros](https://github.com/ouster-lidar/ouster-ros) decoder. It publishes things like LidarScans in a different formats which may be better for things like Lidar Odometry, while things like point clouds, signal, range and IMU remain in the same format. It also has very low latency (<0.2ms) compared to ouster_example (>3ms), tested on Intel i7 11th gen cpu. This will also notify you when packets are dropped. The decoder is up to date with the v0.10 of the ouster SDK.

The decoder only supports LEGACY and single return profile. Currently there's no plan for dual return profile.

## Important

The timestamp of both the cloud and image message is the time of the last column, not the first column.
This is different from the official driver, which uses timestamp of the first column. 

## Setup
Clone this repo in you catkin workspace along with [ouster-ros](https://github.com/ouster-lidar/ouster-ros) and `catkin build`. We use thier custom service and messages and thier ouster sdk submodule. 

## Parameters
The following parameters may differ from the defaults that we use. They can be set in the launch file or passed as an argument.
- `replay` set to `true` if you are using a bag, default: `false`.
- `sensor_hostname` hostname or IP in dotted decimal format of the ouster, default: `192.168.100.12`
- `udp_dest` hostname or IP in dotted decimal format of where the sensor will send data. Most likely the computer the ouster is connected to, default: `192.168.100.1`
- `lidar_port` the port to which the ouster will send lidar packets, default: `7502`
- `imu_port` the port to which the ouster will send imu packets, default: `7503`
- `lidar_mode` resolution and rate of the lidar: either `512x10`, `512x20`, `1024x10`, `1024x20`, or `2048x10`, defualt comes from lidar.
- `timestamp_mode` method used to timestamp measurements: either `IME_FROM_INTERNAL_OSC`, `TIME_FROM_SYNC_PULSE_IN`, `TIME_FROM_PTP_1588`, default comes from lidar.
- `metadata` specifiy a metadata file to write to, default: `ouster_decoder/metadata.json`. This must be specified if you are using a bag without the `/metadata` topic. 
- `tf_prefic` namespace for tf transforms.
- `driver_ns` namespace for driver node.

## Usage
To start everything at once use:
```
roslaunch ouster_decoder driver.launch
```

Run just the driver (if you want to bag the packets for later decoding) 
```
roslaunch ouster_decoder driver.launch
```

To run with a bag file run:
```
roslaunch ouster_decoder ouster.launch replay:=true
```
and start your bag in another terminal. If your bag does not have the `/metadata` topic you'll need to specify a json file with `metadata:=/path/to/json`.

The driver node is the same (logically) as the one from `ouster_example`, but cleaned up and it publishes a string message to topic `/os_node/metadata` that you should also record. When in replay mode, there's no need to specify a metadata file. The metadata file will still be saved in case one forgot to record the metadata message.

## Decoder

The decoder allocates storge for a range image and a point cloud on startup.
When receiving a packet, it fills the range image and the point cloud instead of doing all computations at the end of a sweep. This drastically redueces latency when publishing the cloud/image.

Each pixel in the range image has the following fields:
```
struct Data {
    float x;
    float y;
    float z;
    uint16_t r16u; // range
    uint16_t s16u; // signal
};
```

The corresponding point cloud has point type XYZI.

During each callback, we process the incoming packet and immediately decode and convert to 3d points. When we reach the end of a sweep, the data is directly published without any extra computations.

Therefore, the publish latency of our decoder is typically less than 0.2ms, while the ouster `os_cloud_node` takes more than 3ms to publish a point cloud.

## Data Drops

Our decoder also checks for missing data. This is very useful for debugging purposes. The decoder tracks the frame and column id of each packet and reports any jumps (both forward and backward) if detected.
The decoder then makes sure that missing data is zeroed out in the message. 

Therefore, one can safely stop and replay a recorded rosbag without restarting the driver and decoder.

