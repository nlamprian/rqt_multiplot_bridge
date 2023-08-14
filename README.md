# rqt_multiplot_bridge

This repository contains dockerfiles for using [rqt_multiplot](https://github.com/ANYbotics/rqt_multiplot_plugin) with ROS 2.
The system is brought up by two docker images. One launches rqt_multiplot in ROS 1,
and the other launches a ROS 2 bridge that forwards messages to ROS 1.


## Build

```bash
docker build --tag=rqt_multiplot_bridge:multiplot multiplot
docker build --tag=rqt_multiplot_bridge:dynamic_bridge dynamic_bridge
docker build --tag=rqt_multiplot_bridge:parameter_bridge parameter_bridge
```


## Run

Start by installing `rocker`

```bash
sudo apt install python3-rocker
```

Then launch `rqt_multiplot`

```bash
rocker --x11 --network host rqt_multiplot_bridge:multiplot
```

and finally launch either of the following bridges. The `dynamic_bridge` bridges
all ROS 2 topics, and `parameter_bridge` bridges those topics that have been configured
in the [`/bridge.yaml`](parameter_bridge/bridge.yaml) file.

```bash
rocker --network host rqt_multiplot_bridge:dynamic_bridge
# or
rocker --network host --volume <YOUR_BRIDGE_CONFIG_FILE>:/bridge.yaml -- rqt_multiplot_bridge:parameter_bridge
```

At this point, you should be able to publish ROS 2 topics on your host and have
them picked up by rqt_multiplot.


## Test

The containers run on Ubuntu 20.04 (focal) with ROS Noetic and Galactic, and the
setup has been tested on Ubuntu 22.04 (jammy) with ROS Humble.
