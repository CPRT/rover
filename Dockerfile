############################
# Stage 0: Base Image per Arch
############################
ARG BASE_IMAGE=ubuntu:22.04
FROM cprtsoftware/rover-gstreamer:latest AS gstreamer-build
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
ENV TZ=UTC

# Per-arch APT cache (mount to the real apt cache path; keep per-arch id)
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales apt-utils curl lsb-release gnupg2 libssl-dev \
        software-properties-common build-essential \
        tzdata git pkg-config libgtk-3-0 libavcodec58 libavformat58 libswscale5 \
        libtbb2 libjpeg8 libpng16-16 libtiff5 libdc1394-25\
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN ln -snf /usr/share/zoneinfo/UTC /etc/localtime \
    && echo "UTC" > /etc/timezone

############################
# Stage 1: Minimal ROS2 Base
############################
FROM base AS ros2_humble-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base python3-setuptools python3-wheel \
        libeigen3-dev python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN python3 -m pip install --upgrade pip
COPY requirements.txt .
RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

############################
# Stage 4: Collect package.xml files
############################
FROM alpine:latest AS package_xml_collector
WORKDIR /src
COPY src/ .

RUN find . -name 'package.xml' -exec mkdir -p /collected/{} \; \
    && find . -name 'package.xml' -exec cp {} /collected/{} \;

############################
# Stage 4: OpenCV Build
############################
FROM base AS opencv-build

RUN if ! pkg-config --exists opencv4; then \
      apt-get update && apt-get install -y \
        build-essential cmake git pkg-config \
        libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
        libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
        libdc1394-dev libv4l-dev libopenexr-dev \
        libxvidcore-dev libx264-dev \
        libatlas-base-dev gfortran python3-dev \
        && rm -rf /var/lib/apt/lists/* && \
      echo "OpenCV not found, building from source..." && \
      git clone -b 4.8.0 --depth 1 https://github.com/opencv/opencv.git /opt/opencv && \
      git clone -b 4.8.0 --depth 1 https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib && \
      mkdir -p /opt/opencv/build && cd /opt/opencv/build && \
      cmake -D CMAKE_BUILD_TYPE=Release \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
            -D BUILD_EXAMPLES=OFF \
            -D BUILD_TESTS=OFF \
            -D BUILD_PERF_TESTS=OFF \
            -D WITH_QT=OFF \
            -D WITH_GTK=OFF \
            -D WITH_GTK_2_X=OFF \
            /opt/opencv && \
      make -j"$(nproc)" && make install; \
    else \
      echo "OpenCV already installed, skipping build."; \
    fi

############################
# Stage 5: Minimal Runtime Base
############################
FROM ros2_humble-base AS runtime
COPY --from=gstreamer-build /target/ /
COPY --from=opencv-build /usr/local/ /usr/local/

ENV PATH=/opt/gstreamer/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/gstreamer/lib:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /temporary
# Build kindr
RUN git clone https://github.com/ANYbotics/kindr.git \
    && cd kindr && mkdir build && cd build \
    && cmake .. -DUSE_CMAKE=true \
    && make install \
    && rm -rf /temporary/kindr

COPY --from=package_xml_collector /collected /temporary/src/
RUN source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep init \
    && rosdep update \
    && rosdep install -i -y -r --from-paths src \
    && rm -rf /var/lib/apt/lists/* src /root/.ros/rosdep/*

############################
# Stage 6: Dev Environment
############################
FROM runtime AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
        git x11-apps ros-humble-desktop ros-dev-tools black pylint \
        ros-humble-ament-cmake python3-colcon-common-extensions \
        python3-colcon-ros clang-format cuda-compiler-12-6 \
        cuda-cudart-dev-12-6 cuda-driver-dev-12-6 libpng-dev ccache \
    && rm -rf /var/lib/apt/lists/*

ENV CMAKE_PREFIX_PATH=/opt/ros/humble:/usr/share/eigen3/cmake:$CMAKE_PREFIX_PATH
ENV CMAKE_INCLUDE_PATH=/usr/include/eigen3:$CMAKE_INCLUDE_PATH
ENV CMAKE_LIBRARY_PATH=/opt/ros/humble/lib:$CMAKE_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/opt/ros/humble/lib/pkgconfig:$PKG_CONFIG_PATH

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/profile.d/ros.sh

RUN useradd -ms /bin/bash -u 1000 vscode \
    && usermod -aG zed vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER vscode
RUN sudo chmod -R a+r /usr/local/lib/python3.10/dist-packages/
WORKDIR /
CMD ["/bin/bash"]

############################
# Stage 7: Builder
############################
FROM dev AS builder
WORKDIR /rover
COPY src/ ./src
ARG BUILD_FLAGS=""

ENV CC="ccache gcc"
ENV CXX="ccache g++"
RUN ccache --max-size=1G

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
       --continue-on-error \
       ${BUILD_FLAGS} \
    && rm -rf build log

############################
# Stage 8: Runtime Application
############################
FROM runtime AS rover
WORKDIR /rover
COPY --from=builder /rover/install /rover/install
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/profile.d/ros.sh \
    && echo "if [ -f /rover/install/setup.bash ]; then source /rover/install/setup.bash; fi" >> /etc/profile.d/ros.sh

CMD ["/bin/bash"]
