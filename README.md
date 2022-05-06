# ouster_decoder

This driver/decoder is intended to replace the `ouster_ros` package from
https://github.com/ouster-lidar/ouster_example, providing a ROS1 driver for the
Ouster lidars.

The driver offers a set of
features compared to the manufacturer example driver:

 - Detection of missing data packages: by observing the package sequence number
   we are able to detect lost packages from the sensor and notify the user.

 - Included LiDAR configuration file: no more need for the metadata json file.

The code has been tested in ROS Noetic, and has been tested with firmwares up to 2.1.

## Usage

Run the ouster driver 
```
roslaunch ouster_decoder driver.launch
```

Then run the decoder
```
roslaunch ouster_decoder decoder.launch
```
