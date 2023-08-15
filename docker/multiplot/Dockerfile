FROM ros:noetic-ros-base-focal

RUN export DEBIAN_FRONTEND=noninteractive \
    && apt update && apt upgrade -y \
    && apt install -q -y --no-install-recommends \
        ros-$ROS_DISTRO-rqt-multiplot \
    && rm -rf /var/lib/apt/lists/*

COPY ./multiplot.xml /root/
COPY ./multiplot.launch /

CMD ["roslaunch", "multiplot.launch"]
