FROM osrf/ros:noetic-desktop-full
ENV ROS_DISTRO=noetic

ARG DEBIAN_FRONTEND=noninteractive 

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c"]

# basic installation
RUN apt-get update
RUN apt-get install -y \
    apt-utils \
    git \
    python3 \
    python-is-python3 

# Install workspace
RUN mkdir /catkin_ws
WORKDIR /catkin_ws
RUN mkdir src
RUN mkdir src/arm_challenge
COPY * src/arm_challenge/
RUN git clone https://github.com/iocchi/arm_gazebo.git src/arm_gazebo
RUN ./src/arm_challenge/install_dependencies.sh
RUN source /opt/ros/noetic/setup.bash && catkin_make

ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]