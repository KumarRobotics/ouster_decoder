# ouster_decoder

This decoder is intended to replace the ouster_ros package from https://github.com/ouster-lidar/ouster_example

It has very low latency (<0.2ms) compared to ouster_example (>3ms), tested on Intel i7 11th gen cpu.

The decoder only supports LEGACY and single return profile.

## Usage

Run the ouster driver 
```
roslaunch ouster_decoder driver.launch replay:=true/false
```

Then run the decoder
```
roslaunch ouster_decoder decoder.launch
```

The driver node is the same as the one from `ouster_example` except that it publishes a string message to topic `/os_node/metadata` that you should record. When in replay mode, there's no need to specify a metadata file. The metadata file will still be saved in case one forgets to record the metadata message.

## Decoder

The decoder fills the range image and the point cloud during each packet callback instead of doing all computations at the end of a sweep.
Each pixel in the range image (each point in the point cloud) has the following fields:
```
struct Data {
    float x;
    float y;
    float z;
    uint16_t r16u; // range
    uint16_t s16u; // signal
};
```

During each callback, we process the incoming packet and immediately decode and convert to 3d points. When we reach the end of a sweep, the data is directly published without any extra computations.

Therefore, the publish latency of our decoder is typically less than 0.2ms, while the ouster `os_cloud_node` takes more than 3ms to publish a point cloud.

## Data Drops

Our decoder also checks for missing data. This is very useful for debugging purposes. The decoder tracks the frame and column id of each packet and reports any jumps (both forward and backward) if detected.
The decoder then makes sure that missing data is zeroed out in the message. 
