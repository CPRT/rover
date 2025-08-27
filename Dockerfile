# syntax=docker/dockerfile:1.5

############################
# Stage 0: Base Image per Arch
############################
ARG BASE_IMAGE=ubuntu:22.04
ARG TARGETARCH
FROM ${BASE_IMAGE} AS base

############################
# Stage 1: Minimal ROS 2 Base
############################
FROM base AS ros2_humble-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
ENV TZ=UTC

# Per-arch APT cache (mount to the real apt cache path; keep per-arch id)
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=apt-lists-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        locales apt-utils curl lsb-release gnupg2 \
        software-properties-common build-essential \
        python3-dev tzdata python3-pip git \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN ln -snf /usr/share/zoneinfo/UTC /etc/localtime \
    && echo "UTC" > /etc/timezone

# ROS 2 repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=apt-lists-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base python3-setuptools python3-wheel \
        libeigen3-dev python3-rosdep

# Python dependencies (per-arch pip cache)
COPY requirements.txt .
RUN --mount=type=cache,target=/root/.cache/pip,id=pip-${TARGETARCH},sharing=locked \
    pip3 install -r requirements.txt


############################
# Stage 2: Rust Setup
############################
FROM ros2_humble-base AS rust-builder
ENV PATH="/root/.cargo/bin:${PATH}"
ENV CARGO_HOME=/root/.cargo
ENV RUSTUP_HOME=/root/.rustup
SHELL ["/bin/bash", "-c"]


RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    . "$HOME/.cargo/env" && \
    cargo install cargo-c


############################
# Stage 3: GStreamer Build
############################
FROM rust-builder AS ros2_humble-gstreamer
ARG TARGETARCH


RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=apt-lists-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y \
        zlib1g-dev libffi-dev libssl-dev python3-dev python3-pip \
        flex bison libglib2.0-dev libmount-dev libsrt-openssl-dev \
        build-essential git ninja-build curl ccache 

# Enable compiler caching
ENV CCACHE_DIR=/root/.cache/ccache
ENV PATH="/usr/lib/ccache:${PATH}"
ENV CC="ccache gcc"
ENV CXX="ccache g++"


RUN python3 -m pip install --upgrade --user pip meson

WORKDIR /gstreamer
RUN --mount=type=cache,target=/root/.cache/gstreamer,id=gst-${TARGETARCH} \
    --mount=type=cache,target=/root/.cache/meson,id=meson-${TARGETARCH} \
    --mount=type=cache,target=/root/.cache/ccache,id=ccache-${TARGETARCH},sharing=locked \
    git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git . && \
    git checkout 1.24 && \
    ~/.local/bin/meson setup builddir --prefix=/opt/gstreamer --libdir=lib && \
    ~/.local/bin/meson compile -C builddir && \
    ~/.local/bin/meson install -C builddir --destdir /target

# Build GStreamer Rust plugins
WORKDIR /gst-plugins-rs
ENV LD_LIBRARY_PATH=/target/opt/gstreamer/lib:$LD_LIBRARY_PATH
ENV LIBRARY_PATH=/target/opt/gstreamer/lib:$LIBRARY_PATH
ENV PKG_CONFIG_PATH=/target/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH
SHELL ["/bin/bash", "-c"]

RUN git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git . && \
    cargo cbuild -p gst-plugin-webrtc --prefix=/opt/gstreamer --release && \
    cargo cbuild -p gst-plugin-rtp    --prefix=/opt/gstreamer --release && \
    cargo cinstall -p gst-plugin-webrtc --prefix=/opt/gstreamer --destdir /target --libdir=lib --release && \
    cargo cinstall -p gst-plugin-rtp    --prefix=/opt/gstreamer --destdir /target --libdir=lib --release

############################
# Stage 6: Collect package.xml files
############################
FROM alpine AS package_xml_collector
WORKDIR /src
COPY src/ . 

# Create a directory to hold all package.xml files with relative paths
RUN find . -name 'package.xml' -exec mkdir -p /collected/{} \; \
    && find . -name 'package.xml' -exec cp {} /collected/{} \;

############################
# Stage 7: Minimal Runtime Base
############################
FROM ros2_humble-base AS runtime
COPY --from=ros2_humble-gstreamer /target /

ENV PATH=/opt/gstreamer/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/gstreamer/lib:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN mkdir -p /temporary
WORKDIR /temporary
COPY --from=package_xml_collector /collected /temporary/src/

# rosdep with ROS env loaded (uses cached rosdep/rosdistro)
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=apt-lists-${TARGETARCH},sharing=locked \
    source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep init && rosdep update && \
    rosdep install -i -r -y --from-paths src

RUN git clone https://github.com/ANYbotics/kindr.git && \
    cd kindr && \
    mkdir build && cd build && \
    cmake .. -DUSE_CMAKE=true && \
    make install

############################
# Stage 8: Dev Environment
############################
FROM runtime AS dev
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=apt-lists-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        git x11-apps ros-humble-desktop ros-dev-tools black pylint \
        ros-humble-ament-cmake python3-colcon-common-extensions \
        python3-colcon-ros clang-format cuda-toolkit-12

# Enable compiler caching

RUN useradd -ms /bin/bash -u 1000 vscode && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN chown -R vscode:vscode /usr/local/lib/python3.10/dist-packages
WORKDIR /
CMD ["/bin/bash"]

############################
# Stage 9: Builder
############################
FROM runtime AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG TARGETARCH
ARG DIR=/cprt_rover_24
ARG CUDA_DIR=/usr/local/cuda-12
WORKDIR ${DIR}
ENV CMAKE_PREFIX_PATH=/usr/share/eigen3/cmake:$CMAKE_PREFIX_PATH
ENV CMAKE_INCLUDE_PATH=/usr/include/eigen3:$CMAKE_INCLUDE_PATH
ENV PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
ENV CUDA_TOOLKIT_ROOT_DIR=${CUDA_DIR}
ENV PATH=${CUDA_DIR}/bin:$PATH
ENV LD_LIBRARY_PATH=${CUDA_DIR}/lib64:$LD_LIBRARY_PATH
ENV CCACHE_DIR=/root/.cache/ccache

RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=apt-lists-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop ros-humble-ament-cmake python3-colcon-common-extensions \
        python3-colcon-ros ccache cuda-toolkit-12


COPY src/ ${DIR}/src/

# colcon build with ROS env + ccache
RUN --mount=type=cache,target=/root/.cache/ccache,id=ccache-${TARGETARCH},sharing=locked \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --continue-on-error

############################
# Stage 10: Runtime Application
############################
FROM runtime AS rover
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}
COPY --from=builder ${DIR}/install ${DIR}/install
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/profile.d/ros.sh \
    && echo "if [ -f ${DIR}/install/setup.bash ]; then source ${DIR}/install/setup.bash; fi" >> /etc/profile.d/ros.sh

CMD ["/bin/bash"]
