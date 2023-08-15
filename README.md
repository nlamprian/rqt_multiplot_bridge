# rqt_multiplot_bridge

This package enables using [rqt_multiplot](https://github.com/ANYbotics/rqt_multiplot_plugin)
with ROS 2. The system is brought up by two docker images. One launches rqt_multiplot
in ROS 1, and the other launches a ROS 2 bridge that forwards messages to ROS 1.


## Build

See [here](docker/README.md) for building the docker images.


## Run

Having built the docker images, you may launch `rqt_multiplot` with the following command:

```bash
ros2 launch rqt_multiplot_bridge bringup.launch.py
```
