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
    python-is-python3 \
    dos2unix

# Install workspace
RUN mkdir /catkin_ws
WORKDIR /catkin_ws
RUN mkdir src
# RUN mkdir src/arm_challenge
# COPY * src/arm_challenge/
RUN git clone https://github.com/iocchi/arm_gazebo.git src/arm_gazebo
RUN git clone https://github.com/fagnerpimentel/arm_challenge.git src/arm_challenge
RUN cd src/arm_gazebo && git checkout cfef9ae32c198c8f7c8ba68326ad06ca3d558847
RUN ./src/arm_challenge/install_dependencies.sh
RUN source /opt/ros/noetic/setup.bash && catkin_make

# ADD ros-entrypoint.sh /ros-entrypoint.sh
# ENTRYPOINT [ "/catkin_ws/src/arm_challenge/ros-entrypoint.sh" ]
