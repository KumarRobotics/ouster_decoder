# ouster_decoder

This decoder is intended to extend the ouster_ros package from https://github.com/ouster-lidar/ouster_example

It has very low latency (<0.2ms) compared to ouster_example (>3ms), tested on Intel i7 11th gen cpu.

The decoder only supports LEGACY and single return profile. Currently there's no plan for dual return profile.

## Important

The timestamp of both the cloud and image message is the time of the last column, not the first column.
This is different from the official driver, which uses timestamp of the first column. 

## Usage

Run the ouster driver 
```
roslaunch ouster_decoder driver.launch replay:=true/false
```

Then run the decoder
```
roslaunch ouster_decoder decoder.launch
```
Running on hardware requires setting a few more parameters:
```
roslaunch ouster_decoder driver.launch sensor_hostname:=192.168.100.12 lidar_port:=7502 imu_port:=7503 udp_dest:=192.168.100.1 replay:=false
```
The driver node is the same as the one from `ouster_example` except that it publishes a string message to topic `/os_node/metadata` that you should also record. When in replay mode, there's no need to specify a metadata file. The metadata file will still be saved in case one forgot to record the metadata message.

## Decoder

The decoder allocates storge for a range image and a point cloud on startup.
When receiving a packet, it fills the range image and the point cloud instead of doing all computations at the end of a sweep. This drastically redueces latency when publishing the cloud/image.

Each pixel in the range image has the following fields:
```
struct Data {
    float x;
    float y;
    float z;
    uint32_t range; // range
    uint32_t signal; // signal
};
```

The corresponding point cloud has point type XYZI.

During each callback, we process the incoming packet and immediately decode and convert to 3d points. When we reach the end of a sweep, the data is directly published without any extra computations.

Therefore, the publish latency of our decoder is typically less than 0.2ms, while the ouster `os_cloud_node` takes more than 3ms to publish a point cloud.

## Data Drops

Our decoder also checks for missing data. This is very useful for debugging purposes. The decoder tracks the frame and column id of each packet and reports any jumps (both forward and backward) if detected.
The decoder then makes sure that missing data is zeroed out in the message. 

Therefore, one can safely stop and replay a recorded rosbag without restarting the driver and decoder.

## Notes
CPU usage is 9.0 percent with ouster_ros.
