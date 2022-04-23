# ouster_decoder

This decoder is intended to replace the ouster_ros package from https://github.com/ouster-lidar/ouster_example

It has very low latency (<1ms) compared to ouster_example (>8ms).

## Usage

Run the ouster driver 
```
roslaunch ouster_decoder driver.launch
```

Then run the decoder
```
roslaunch ouster_decoder decoder.launch
```
