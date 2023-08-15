FROM ros:galactic-ros1-bridge

COPY ./bridge_entrypoint.sh /
COPY ./bridge.yaml /

ENTRYPOINT ["/bridge_entrypoint.sh"]
CMD ["ros2", "run", "ros1_bridge", "parameter_bridge"]
