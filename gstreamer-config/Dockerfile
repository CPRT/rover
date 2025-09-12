ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE} AS gstreamer-build
# Versions:
ARG gstreamer_version=1.24.13


ENV PATH="/root/.cargo/bin:${PATH}"
ENV CARGO_HOME=/root/.cargo
ENV RUSTUP_HOME=/root/.rustup
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl lsb-release gnupg2 libssl-dev \
        software-properties-common \
        git pkg-config libpng-dev \
        zlib1g-dev libffi-dev python3-dev python3-pip \
        flex bison libglib2.0-dev libmount-dev libsrt-openssl-dev \
        build-essential ninja-build \
    && rm -rf /var/lib/apt/lists/*

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    . "$HOME/.cargo/env" && \
    cargo install cargo-c

RUN python3 -m pip install --upgrade --user pip meson
ENV PATH="/root/.local/bin:${PATH}"

# Build GStreamer from source
WORKDIR /gstreamer
RUN git clone --branch ${gstreamer_version} --single-branch --depth 1 https://gitlab.freedesktop.org/gstreamer/gstreamer.git . \
    && meson setup builddir --prefix=/opt/gstreamer --libdir=lib --buildtype=release \
    && meson compile -C builddir \
    && meson install -C builddir --destdir /target \
    && rm -rf builddir


# Build GStreamer Rust plugins we need from source
WORKDIR /gst-plugins-rs
ENV LD_LIBRARY_PATH=/target/opt/gstreamer/lib:$LD_LIBRARY_PATH
ENV LIBRARY_PATH=/target/opt/gstreamer/lib:$LIBRARY_PATH
ENV PKG_CONFIG_PATH=/target/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH

RUN git clone --branch gstreamer-${gstreamer_version} --depth 1 https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git . \
    && cargo cbuild -p gst-plugin-webrtc --prefix=/opt/gstreamer --release \
    && cargo cbuild -p gst-plugin-rtp    --prefix=/opt/gstreamer --release \
    && cargo cinstall -p gst-plugin-webrtc --prefix=/opt/gstreamer --destdir /target --libdir=lib --release \
    && cargo cinstall -p gst-plugin-rtp    --prefix=/opt/gstreamer --destdir /target --libdir=lib --release \
    # Remove static libs to save space
    && rm -f /target/opt/gstreamer/lib/gstreamer-1.0/*.a \
    && rm -rf ./target

############################
# Final Stage: Copy to final image
############################
FROM alpine:latest
COPY --from=gstreamer-build /target /target
