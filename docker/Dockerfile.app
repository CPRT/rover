############################
# Builder
############################
ARG DEV_IMAGE=cprtsoftware/rover:dev
ARG BASE_IMAGE=cprtsoftware/rover:base
FROM ${DEV_IMAGE} AS builder
WORKDIR /rover
COPY src/ ./src
COPY aarch64_toolchain.cmake ./aarch64_toolchain.cmake
ARG BUILD_FLAGS=""

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --continue-on-error \
       ${BUILD_FLAGS}

############################
# Runtime Application
############################
FROM ${BASE_IMAGE} AS rover
WORKDIR /rover
COPY --from=builder /rover/install /rover/install
COPY --from=builder /rover/build /rover/build
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /ros.sh \
    && echo "if [ -f /rover/install/setup.bash ]; then source /rover/install/setup.bash; fi" >> /ros.sh
RUN chmod +x /ros.sh
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /ros.sh && exec \"$@\"", "--"]

CMD ["/bin/bash"]


#############################
# Linter (Optional)
#############################
FROM builder AS linter
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /rover
COPY ros.sh .
COPY .pylintrc .
RUN black . --exclude "src/third-party/|build|install|\.tox|dist" --check
RUN find ./src -path ./src/third-party -prune -o \
    \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -print \
    | xargs clang-format --dry-run --Werror
RUN source ros.sh && pylint -E src

############################
# Rosdep collector (Optional)
############################
FROM builder AS rosdep-collector
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /rover
COPY src/ ./src
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && rosdep update && \
    rosdep keys --from-paths src --ignore-src > /rosdep-keys.txt
RUN sort /rosdep-keys.txt -o /rosdep-keys.txt

############################
# Rosdep exporter (Optional)
############################
FROM scratch AS rosdep-exporter
COPY --from=rosdep-collector /rosdep-keys.txt .