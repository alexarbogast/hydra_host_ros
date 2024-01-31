ARG ROS_DISTRO=noetic

############################
# base
############################
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y --no-install-recommends \
  sudo \
  bash-completion \
  ros-dev-tools \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

############################
# dev
############################
FROM base AS dev 

RUN apt-get update && apt-get install -y --no-install-recommends \
  neovim \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-ros-control \
  ros-${ROS_DISTRO}-ros-controllers \
  ros-${ROS_DISTRO}-ros-control-boilerplate \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  && rm -rf /var/lib/apt/lists/*
  
ENV TERM=xterm-256color
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc

