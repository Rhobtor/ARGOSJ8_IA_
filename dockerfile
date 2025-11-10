FROM ubuntu:22.04

# 1) Variables de entorno ------------------------------------------------------
ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    SETUPTOOLS_USE_DISTUTILS=stdlib

# 2) Locales, Qt 6 y básicos ---------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        locales \
        curl gnupg lsb-release ca-certificates software-properties-common \
        build-essential python3-dev python3-pip python3-distutils \
        libgl1-mesa-glx libglib2.0-0 \
        libqt6gui6 libqt6widgets6 libqt6core6 qt6-base-dev && \
    locale-gen en_US.UTF-8 && \
    add-apt-repository -y universe && \
    rm -rf /var/lib/apt/lists/*

# 3) Repositorio ROS 2 Humble --------------------------------------------------
RUN install -d /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros2.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros2.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# 4) Repositorio OSRF (Gazebo 11) ---------------------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      | gpg --dearmor -o /etc/apt/keyrings/osrf.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/osrf.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# 5) ROS 2 + deps de sistema (sin LCM, lo compilamos nosotros) -----------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop-full \
        python3-colcon-common-extensions python3-vcstool python3-rosdep \
        gazebo libgazebo-dev \
        geographiclib-tools libgeographic-dev \
        libopencv-dev libpcl-dev libnanoflann-dev \
        libxcb-cursor0 libxkbcommon-x11-0 libxcb-xinerama0 \  
        git cmake ninja-build build-essential libglib2.0-dev && \
    rosdep init && rosdep update && \
    rm -rf /var/lib/apt/lists/*

# 5.b) Compilar e instalar LCM 1.5 (aporta lcmConfig.cmake) --------------------
RUN git clone --depth 1 --branch v1.5.0 https://github.com/lcm-proj/lcm.git /tmp/lcm && \
    cmake -S /tmp/lcm -B /tmp/lcm/build -G Ninja \
          -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release && \
    cmake --build  /tmp/lcm/build && \
    cmake --install /tmp/lcm/build && \
    ldconfig && rm -rf /tmp/lcm

# 6) Entorno Python coherente + deps GUI ---------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade \
        pip setuptools~=69.5 packaging wheel && \
    python3 -m pip install --no-cache-dir \
        "numpy<1.24" \
        "PySide6~=6.5" \
        Cython~=0.29 \
        utm pyproj

# 7) Copiar código y compilar el workspace -------------------------------------
WORKDIR /ros2_ws
COPY src ./src

RUN apt-get update && \
    bash -c "set -e \
      && source /opt/ros/${ROS_DISTRO}/setup.bash \
      && rosdep update \
      && rosdep install --from-paths src --ignore-src -r -y \
      && colcon build --symlink-install"

# 8) Entrypoint y comando por defecto ------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
