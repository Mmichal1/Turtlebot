FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get -y upgrade \
    && apt-get install -y \
    vim \
    software-properties-common \
    python3-pip \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
&& curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update \
    && apt-get install -y \
    ament-cmake \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-cartographer \
    ros-$ROS_DISTRO-cartographer-ros \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/bashrc /home/${USERNAME}/.bashrc



WORKDIR /home/ros/turtlebot3/

RUN colcon build --symlink-install

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]