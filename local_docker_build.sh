#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="cprtsoftware/rover"
GIT_SHA=$(git rev-parse HEAD)

# Detect local architecture
ARCH=$(uname -m)
case "$ARCH" in
  x86_64)
    TARGETARCH=amd64
    BASE_IMAGE="stereolabs/zed:4.2-runtime-cuda12.1-ubuntu22.04"
    ;;
  aarch64)
    TARGETARCH=arm64
    BASE_IMAGE="stereolabs/zed:4.2-runtime-l4t-r36.4"
    ;;
  *)
    echo "Unsupported architecture: $ARCH"
    exit 1
    ;;
esac

echo "Building for local architecture: $TARGETARCH (base: $BASE_IMAGE)"

# Ensure buildx is set up
docker buildx create --use --name roverbuilder 2>/dev/null || true

# --- DEV image ---
docker buildx build \
  --build-arg BASE_IMAGE=$BASE_IMAGE \
  --build-arg TARGETARCH=$TARGETARCH \
  -f Dockerfile \
  --target dev \
  -t ${IMAGE_NAME}:dev-${TARGETARCH} \
  --cache-from=type=registry,ref=${IMAGE_NAME}:${TARGETARCH}-cache \
  --cache-to=type=registry,ref=${IMAGE_NAME}:${TARGETARCH}-cache,mode=max \
  --load \
  .

# --- APP (rover) image ---
docker buildx build \
  --build-arg BASE_IMAGE=$BASE_IMAGE \
  --build-arg TARGETARCH=$TARGETARCH \
  -f Dockerfile \
  --target builder \
  -t ${IMAGE_NAME}:${TARGETARCH} \
  --cache-from=type=registry,ref=${IMAGE_NAME}:${TARGETARCH}-cache \
  --cache-to=type=registry,ref=${IMAGE_NAME}:${TARGETARCH}-cache,mode=max \
  --load \
  .
