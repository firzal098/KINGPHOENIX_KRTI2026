ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
ENV RCUTILS_COLORIZED_OUTPUT=1

# Install common development, build, networking tools, and MAVROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-argcomplete \
    wget \
    curl \
    nano \
    tmux \
    bash-completion \
    iputils-ping \
    net-tools \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    mesa-utils \
    udev \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets required for MAVROS coordinate conversions
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 run mavros install_geographiclib_datasets.sh"

# Initialize rosdep if needed and update database
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Create ROS 2 workspace structure
WORKDIR /ros_ws
RUN mkdir -p /ros_ws/src

# Set environment variable so non-interactive bash commands (bash -c) also source the ROS entrypoint
ENV BASH_ENV=/ros_entrypoint.sh

# Set up environment sourcing, bash completion, and convenience aliases
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo 'if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi' >> /etc/bash.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc && \
    echo "alias cb='cd /ros_ws && colcon build --symlink-install && source /ros_ws/install/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias sb='source /ros_ws/install/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias ws='cd /ros_ws'" >> /etc/bash.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
    echo "alias cb='cd /ros_ws && colcon build --symlink-install && source /ros_ws/install/setup.bash'" >> /root/.bashrc && \
    echo "alias sb='source /ros_ws/install/setup.bash'" >> /root/.bashrc && \
    echo "alias mavros-sitl='ros2 launch mavros apm.launch fcu_url:=tcp://172.24.123.183:5760 gcs_url:=udp://@172.24.112.1:14550 tgt_system:=1 tgt_component:=1'" >> /root/.bashrc && \
    echo "alias ws='cd /ros_ws'" >> /root/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gscam \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    tcpdump \
    && rm -rf /var/lib/apt/lists/*
    
ENV GSCAM_CONFIG="udpsrc port=5599 caps=\"image/jpeg\" ! jpegdec ! videoconvert"

RUN apt-get install -y tcpdump

# Copy and setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN echo "v1.7"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]