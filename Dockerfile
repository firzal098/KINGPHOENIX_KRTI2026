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
    python3.12-venv \
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

# Copy and setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Install Foxglove Bridge, ROSBridge Suite, vision_msgs, Node.js, and npm
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-vision-msgs \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*


# ── Custom Python libraries ───────────────────────────────────────────────────
# Install into the SYSTEM Python so ros2 run / ros2 launch nodes can import
# them directly. Using --break-system-packages is safe here because this is
# an isolated Docker container (PEP 668 doesn't apply).
COPY src/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        -r /tmp/requirements.txt

RUN echo "v2.0"

# ── CUDA 13 runtime + cuDNN installation ────────────────────────────────────
ARG INSTALL_CUDA=false
RUN if [ "$INSTALL_CUDA" = "true" ]; then \
    apt-get update && apt-get install -y --no-install-recommends wget ca-certificates gnupg && \
    wget -qO /tmp/cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-13-3 \
        cuda-nvrtc-13-3 \
        libcublas-13-3 \
        libcurand-13-3 \
        libcufft-13-3 \
        libcudnn9-cuda-13 && \
    echo "/usr/local/cuda/lib64" > /etc/ld.so.conf.d/cuda.conf && \
    echo "/usr/local/cuda-13.3/lib64" >> /etc/ld.so.conf.d/cuda.conf && \
    echo "/usr/local/cuda-13.3/targets/x86_64-linux/lib" >> /etc/ld.so.conf.d/cuda.conf && \
    ldconfig; \
fi

# ── Separate cleanup step ───────────────────────────────────────────────────
ARG INSTALL_CUDA=false
RUN if [ "$INSTALL_CUDA" = "true" ]; then \
    rm -rf /usr/local/cuda*/include /usr/share/doc /usr/share/man /var/lib/apt/lists/*; \
fi

ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda-13.3/lib64:/usr/local/cuda-13.3/targets/x86_64-linux/lib:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}


# ── ONNX Runtime C++ Installation ──────────────────────────────────────────
ARG ONNXRUNTIME_VERSION=1.18.0
ARG ONNXRUNTIME_ARCH=auto

RUN ARCH=$(uname -m) && \
    if [ "$ONNXRUNTIME_ARCH" != "auto" ]; then \
        ORT_ARCH="$ONNXRUNTIME_ARCH"; \
    elif [ "$ARCH" = "x86_64" ]; then \
        ORT_ARCH="x64"; \
    elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then \
        ORT_ARCH="aarch64"; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi && \
    echo "Installing ONNX Runtime v${ONNXRUNTIME_VERSION} for ${ORT_ARCH}..." && \
    wget -q https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}.tgz -O /tmp/ort.tgz && \
    tar -xzf /tmp/ort.tgz -C /tmp && \
    cp -r /tmp/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}/include/* /usr/local/include/ && \
    cp -r /tmp/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}/lib/* /usr/local/lib/ && \
    ldconfig && \
    rm -rf /tmp/ort.tgz /tmp/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]