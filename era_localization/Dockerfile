FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive 

# Install necessary packages
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

RUN apt update && apt install -y vim wget

# Set up ROS 2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2.list && \
    apt update

# Install ROS 2 Humble and dependencies
RUN apt install -y \
    ros-humble-ros-base \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    git && \
    rm -rf /var/lib/apt/lists/*
# Install Gazebo and gazebo_ros packages
RUN apt update && apt install -y \
    gazebo \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install xacro package
RUN apt update && apt install -y ros-humble-xacro

# Install RViz
RUN apt update && apt install -y ros-humble-rviz2

# Install RViz
RUN apt update && apt install -y tmux

# Set up workspace
WORKDIR /root/ros2_ws
COPY src /root/ros2_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source the setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws1
COPY src1 /root/ros2_ws1

RUN /bin/bash -c "source /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "source /root/ros2_ws1/install/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws2
COPY src2 /root/ros2_ws2

RUN /bin/bash -c "source /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "source /root/ros2_ws2/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
