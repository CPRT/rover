#!/usr/bin/env bash
set -euo pipefail

# Load config
source config.env

GIT_SHA=$(git rev-parse HEAD)

# Default settings
MODE="--load"
OVERRIDE_ARCH=""
GST=FALSE
BUILD_WORKERS=$(nproc)
BUILD_TYPE="release"

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --push)
      MODE="--push"
      shift
      ;;
    --load)
      MODE="--load"
      shift
      ;;
    --arch)
      OVERRIDE_ARCH="$2"
      shift 2
      ;;
    --gstreamer)
      GST=TRUE
      shift
      ;;
    --workers)
      BUILD_WORKERS="$2"
      shift 2
      ;;
    --build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    *)
      echo "Usage: $0"
      echo "[--push|--load (default)]"
      echo "[--arch <amd64|arm64> (optional for emulation)]"
      echo "[--gstreamer (to build gstreamer image first)]"
      echo "[--workers <num> (default: Number of cores in system {$(nproc)})]"
      echo "[--build-type <release|debug> (default: release)]"
      exit 1
      ;;
  esac
done

# Detect local architecture (or override)
ARCH="${OVERRIDE_ARCH:-$(uname -m)}"
case "$ARCH" in
  x86_64|amd64)
    TARGETARCH=amd64
    BASE_IMAGE=$ZED_AMD_IMAGE
    ;;
  aarch64|arm64)
    TARGETARCH=arm64
    BASE_IMAGE=$ZED_ARM_IMAGE
    ;;
  *)
    echo "Unsupported architecture: $ARCH"
    exit 1
    ;;
esac

echo "Building for architecture: $TARGETARCH with mode $MODE"

# --- Create a persistent buildx builder with QEMU if not exists ---
BUILDER_NAME="roverbuilder"
if ! docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
  echo "Creating persistent buildx builder '$BUILDER_NAME' with QEMU support..."
  docker buildx create --name "$BUILDER_NAME" --use --driver docker-container --bootstrap
  # Register QEMU emulators
  docker run --rm --privileged tonistiigi/binfmt --install all
else
  docker buildx use "$BUILDER_NAME"
fi

if [ "$GST" = TRUE ]; then
  echo "Building GStreamer image first..."
  ./gstreamer-config/docker_build.sh $MODE --arch $TARGETARCH
fi

# Define tags
DEV_TAG=${IMAGE_NAME}:dev-${TARGETARCH}
DEV_SHA_TAG=${IMAGE_NAME}:dev-${GIT_SHA}-${TARGETARCH}
APP_TAG=${IMAGE_NAME}:${TARGETARCH}
APP_SHA_TAG=${IMAGE_NAME}:${GIT_SHA}-${TARGETARCH}

# --- DEV image ---
echo "Starting dev image (base: $BASE_IMAGE)"
docker buildx build \
  --build-arg BASE_IMAGE=$BASE_IMAGE \
  -f Dockerfile \
  --platform linux/$TARGETARCH \
  --target dev \
  -t $DEV_TAG \
  -t $DEV_SHA_TAG \
  --cache-from type=registry,ref=$IMAGE_NAME:$TARGETARCH-dev-cache \
  --cache-to type=registry,ref=$IMAGE_NAME:$TARGETARCH-dev-cache,mode=max \
  $MODE \
  .

# --- APP (rover) image ---
echo "Starting rover image (base: $BASE_IMAGE)"
docker buildx build \
  --build-arg BASE_IMAGE=$BASE_IMAGE \
  -f Dockerfile \
  --target rover \
  --platform linux/$TARGETARCH \
  -t $APP_TAG \
  -t $APP_SHA_TAG \
  --build-arg BUILD_FLAGS="--parallel-workers $BUILD_WORKERS\
              --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
  --cache-from type=registry,ref=$IMAGE_NAME:$TARGETARCH-cache \
  --cache-to type=registry,ref=$IMAGE_NAME:$TARGETARCH-cache,mode=max \
  $MODE \
  .
