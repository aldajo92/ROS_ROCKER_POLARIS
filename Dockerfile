FROM osrf/ros:noetic-desktop-full

RUN mkdir -p /catkin_ws
# WORKDIR /catkin_ws

RUN apt update && apt-get install -y \
  ros-noetic-ackermann-msgs ros-noetic-geometry2 \
  ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
  ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator

RUN apt update && apt-get install -y \
  ros-noetic-catkin python3-catkin-tools

RUN apt update && apt-get install -y \
  ros-noetic-rqt-plot

COPY ./autostart.sh /
RUN chmod +x /autostart.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
 