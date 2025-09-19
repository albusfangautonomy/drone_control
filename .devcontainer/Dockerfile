# ========================
# Base: Ubuntu 22.04 ARM64
# ========================
FROM --platform=linux/arm64 ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DISTRO=humble \
    PX4_DIR=/opt/PX4-Autopilot

# ---------- Core OS deps ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata sudo bash-completion \
    ca-certificates gnupg lsb-release wget curl git \
    build-essential cmake ninja-build ccache gdb rsync zip unzip \
    python3 python3-pip python3-venv python3-jinja2 \
    xz-utils file pkg-config \
    mesa-utils libglu1-mesa libgl1-mesa-glx libglfw3 \
    && rm -rf /var/lib/apt/lists/*

# ---------- Locale ----------
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---------- Add ROS 2 apt repo (arch-aware) ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    ARCH=$(dpkg --print-architecture) && \
    echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# ---------- Add Gazebo (Fortress) apt repo (arch-aware) ----------
# RUN wget -qO /etc/apt/keyrings/gazebo-archive-keyring.gpg \
#       https://packages.osrfoundation.org/gazebo.gpg && \
#     ARCH=$(dpkg --print-architecture) && \
#     echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] \
#       http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
#       > /etc/apt/sources.list.d/gazebo-stable.list

# ---------- Add Gazebo (Harmonic) apt repo per OSRF docs ----------
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends curl lsb-release gnupg ca-certificates; \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# ---------- Install Gazebo Harmonic meta package ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*


# ---------- Install ROS 2 Humble + tools ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# ---------- Install MAVROS2 + GeographicLib datasets ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros ros-humble-mavros-extras geographiclib-tools \
    && geographiclib-get-geoids egm96-5 && geographiclib-get-gravity egm2008 \
    && rm -rf /var/lib/apt/lists/*

# ---------- Install MAVSDK (Python bindings) ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libprotobuf-dev protobuf-compiler libgrpc++-dev libgrpc-dev \
    && pip3 install --no-cache-dir mavsdk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${PX4_DIR}

# RUN cd ${PX4_DIR} && bash Tools/setup/ubuntu.sh --no-nuttx --gazebo || true
# Reclone PX4 v1.16
RUN git clone https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}" && \
    cd "${PX4_DIR}" && \
    git checkout v1.16.0 && \
    git submodule update --init --recursive && \
    PX4_NO_NUTTX=1 bash Tools/setup/ubuntu.sh --no-nuttx

RUN echo 'if [ -f /usr/share/gazebo/setup.sh ]; then source /usr/share/gazebo/setup.sh; fi' >> /etc/bash.bashrc || true
RUN echo 'if [ -f /usr/share/ignition/setup.sh ]; then source /usr/share/ignition/setup.sh; fi' >> /etc/bash.bashrc || true
RUN echo 'if [ -f /usr/share/gz/setup.sh ]; then source /usr/share/gz/setup.sh; fi' >> /etc/bash.bashrc || true

# ---------- PX4 SITL (Gazebo) ----------


# RUN cd ${PX4_DIR} && bash Tools/setup/ubuntu.sh --no-nuttx --gazebo || true
# RUN cd ${PX4_DIR} && DONT_RUN=1 make px4_sitl gz_x500

# ---------- Micro XRCE-DDS Agent (append-only block) ----------
# Extra deps that aren't already in your base:
RUN apt-get update && apt-get install -y --no-install-recommends \
    libasio-dev libtinyxml2-dev \
 && rm -rf /var/lib/apt/lists/*

# Build & install Agent
WORKDIR /opt
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig
RUN pip install rerun-sdk
# Convenience runner (defaults to UDP 8888; override with XRCE_PORT)
RUN printf '#!/usr/bin/env bash\n' \
           'set -e\n' \
           'exec MicroXRCEAgent udp4 -p "${XRCE_PORT:-8888}"\n' \
           > /usr/local/bin/xrce-agent-start && chmod +x /usr/local/bin/xrce-agent-start
# ---------- Convenience scripts ----------
RUN mkdir -p /root/.px4/log /root/logs

RUN printf '#!/usr/bin/env bash\n' \
           'set -e\n' \
           'source /opt/ros/${ROS_DISTRO}/setup.bash\n' \
           'if [ -f /usr/share/ignition/setup.sh ]; then source /usr/share/ignition/setup.sh; fi\n' \
           'if [ -f /usr/share/gz/setup.sh ]; then source /usr/share/gz/setup.sh; fi\n' \
           'export PX4_SIM_SPEED_FACTOR=1.0\n' \
           'export PX4_LOG_DIR=/root/logs\n' \
           'cd '"${PX4_DIR}"'\n' \
           'echo "Starting PX4 SITL (gz_x500) on UDP 14540..."\n' \
           'make px4_sitl gz_x500\n' \
           > /usr/local/bin/px4-sitl-gz && chmod +x /usr/local/bin/px4-sitl-gz

RUN printf '#!/usr/bin/env bash\n' \
           'set -e\n' \
           'source /opt/ros/${ROS_DISTRO}/setup.bash\n' \
           'ROS_DOMAIN_ID=${ROS_DOMAIN_ID:=0}\n' \
           'echo "Starting MAVROS2 bound to PX4 udp://:14540"\n' \
           'ros2 launch mavros mavros.launch.py fcu_url:=udp://:14540@ localhost:=127.0.0.1 tgt_system:=${MAV_SYSID:=1} tgt_component:=1\n' \
           > /usr/local/bin/mavros2-start && chmod +x /usr/local/bin/mavros2-start

RUN printf '#!/usr/bin/env python3\n' \
           'import asyncio\n' \
           'from mavsdk import System\n' \
           'async def main():\n' \
           '    drone = System()\n' \
           '    await drone.connect(system_address="udp://:14540")\n' \
           '    print("Waiting for connection...")\n' \
           '    async for state in drone.core.connection_state():\n' \
           '        if state.is_connected:\n' \
           '            print("Connected!")\n' \
           '            break\n' \
           '    print("Heartbeat OK.")\n' \
           'asyncio.run(main())\n' \
           > /usr/local/bin/mavsdk-test && chmod +x /usr/local/bin/mavsdk-test
WORKDIR /opt/PX4-Autopilot
CMD ["source /opt/ros/humble/setup.bash"]
# CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash" ]
#  && if [ -f /usr/share/gz/setup.sh ]; then source /usr/share/gz/setup.sh; fi \
#  && if [ -f /usr/share/ignition/setup.sh ]; then source /usr/share/ignition/setup.sh; fi \
#  && cd ${PX4_DIR} \
#  && make px4_sitl gz_x500"]
