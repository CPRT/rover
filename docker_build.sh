#!/usr/bin/env bash
set -euo pipefail

# Load config
IMAGE_NAME="cprtsoftware/rover"
BASE_IMAGE=nvidia/cuda:13.3.0-cudnn-runtime-ubuntu24.04
GIT_SHA=$(git rev-parse HEAD)

# Default settings
MODE="--load"
OVERRIDE_ARCH=""
BUILD_TYPE="release"
LITE=FALSE
LOCAL=FALSE

BUILD_BASE_IMAGE=TRUE
BUILD_DEV_IMAGE=TRUE
BUILD_APP_IMAGE=TRUE

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
    --test)
      MODE="--test"
      BUILD_BASE_IMAGE=FALSE
      BUILD_DEV_IMAGE=FALSE
      BUILD_APP_IMAGE=FALSE
      shift
      ;;
    --arch)
      OVERRIDE_ARCH="$2"
      shift 2
      ;;
    --build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    --lite)
      LITE=TRUE
      shift
      ;;
    --dev)
      if [[ "$2" == "true" ]]; then
        BUILD_DEV_IMAGE=TRUE
      elif [[ "$2" == "false" ]]; then
        BUILD_DEV_IMAGE=FALSE
      else
        echo "Invalid value for --dev. Use true or false."
        exit 1
      fi
      shift 2
      ;;
    --app)
      if [[ "$2" == "true" ]]; then
        BUILD_APP_IMAGE=TRUE
      elif [[ "$2" == "false" ]]; then
        BUILD_APP_IMAGE=FALSE
      else
        echo "Invalid value for --app. Use true or false."
        exit 1
      fi
      shift 2
      ;;
    --base)
      if [[ "$2" == "true" ]]; then
        BUILD_BASE_IMAGE=TRUE
      elif [[ "$2" == "false" ]]; then
        BUILD_BASE_IMAGE=FALSE
      else
        echo "Invalid value for --base. Use true or false."
        exit 1
      fi
      shift 2
      ;;
    --local)
      LOCAL=TRUE
      shift
      ;;
    *)
      echo "Usage: $0"
      echo "[--push|--load (default)|--test (runs lint and build)]"
      echo "[--arch <amd64|arm64|both> (default: auto-detect) (both only supported with --push)]"
      echo "[--build-type <release|debug> (default: release)]"
      echo "[--lite (builds a smaller image without nvidia, and ZED dependencies)]"
      echo "[--dev (true | false) (default: true) (whether to build the dev image)]"
      echo "[--app (true | false) (default: true) (whether to build the app image)]"
      echo "[--base (true | false) (default: true) (whether to build the base image)]"
      echo "[--local (builds only the app from src without any installs and can be used without internet access requires that dev and base images are already available locally)]"
      exit 1
      ;;
  esac
done

# Detect local architecture (or override)
ARCH="${OVERRIDE_ARCH:-$(uname -m)}"
case "$ARCH" in
  x86_64|amd64)
    TARGETARCH=amd64
    ;;
  aarch64|arm64)
    TARGETARCH=arm64
    ;;
  both)
    TARGETARCH=both
    ;;
  *)
    echo "Unsupported architecture: $ARCH"
    exit 1
    ;;
esac

if [ "$TARGETARCH" = "both" ] && [ "$MODE" != "--push" ]; then
  echo "Error: --both architecture is only supported with --push mode"
  exit 1
fi

echo "Building for architecture: $TARGETARCH with mode $MODE"

# --- Create a persistent buildx builder with QEMU if not exists ---
docker run --rm --privileged tonistiigi/binfmt --install all

# Define tags
BASE_TAG=${IMAGE_NAME}:base
BASE_SHA_TAG=${IMAGE_NAME}:base-${GIT_SHA}
DEV_TAG=${IMAGE_NAME}:dev
DEV_SHA_TAG=${IMAGE_NAME}:dev-${GIT_SHA}
APP_TAG=${IMAGE_NAME}:latest
APP_SHA_TAG=${IMAGE_NAME}:${GIT_SHA}

PLATFORM_ARG=""
if [ "$TARGETARCH" = "both" ]; then
  PLATFORM_ARG="--platform linux/amd64,linux/arm64"
else
  PLATFORM_ARG="--platform linux/$TARGETARCH"
fi


TOOLCHAIN="-DCMAKE_TOOLCHAIN_FILE=/rover/toolchain.cmake"
BUILD_FLAGS="--cmake-args $TOOLCHAIN -DCMAKE_BUILD_TYPE=$BUILD_TYPE"

ADDITIONAL_BUILD_ARGS=""
STARTING_IMAGE_ARG=""
DEV_TARGET="dev"
if [ "$LITE" = "TRUE" ]; then
  echo "Building lite image without nvidia and ZED dependencies"
  BASE_IMAGE=ubuntu:24.04
  PACKAGES_TO_SKIP="zed_components zed_ros2 zed_wrapper zed_debug video_streaming"
  BUILD_FLAGS="$BUILD_FLAGS --packages-skip $PACKAGES_TO_SKIP"
  ADDITIONAL_BUILD_ARGS="--build-arg USE_ZED_SDK=false"
  ADDITIONAL_BUILD_ARGS="$ADDITIONAL_BUILD_ARGS --build-arg USE_NVIDIA=false"
  ADDITIONAL_BUILD_ARGS="$ADDITIONAL_BUILD_ARGS --build-arg BASE_IMAGE=ubuntu:24.04"
  BASE_TAG=$BASE_TAG-lite
  BASE_SHA_TAG=$BASE_SHA_TAG-lite
  DEV_TAG=$DEV_TAG-lite
  DEV_SHA_TAG=$DEV_SHA_TAG-lite
  APP_TAG=$APP_TAG-lite
  APP_SHA_TAG=$APP_SHA_TAG-lite
  DEV_TARGET="dev-lite"
fi


if [ "$BUILD_BASE_IMAGE" = "TRUE" ]; then
  echo "Building base image..."
  docker buildx build \
    -f docker/Dockerfile.base \
    $ADDITIONAL_BUILD_ARGS \
    --target runtime \
    $PLATFORM_ARG \
    -t $BASE_TAG \
    -t $BASE_SHA_TAG \
    $MODE \
    .
elif [ "$LOCAL" = "FALSE" ]; then
  echo "Pulling base image from registry..."
  docker pull $BASE_TAG || true
fi

# # --- DEV image ---
if [ "$BUILD_DEV_IMAGE" = "TRUE" ]; then
  echo "Starting dev image (base: $BASE_TAG)"
  docker buildx build \
    --build-arg BASE_IMAGE=$BASE_TAG \
    -f docker/Dockerfile.dev \
    $PLATFORM_ARG \
    --target $DEV_TARGET \
    -t $DEV_TAG \
    -t $DEV_SHA_TAG \
    $MODE \
    .
elif [ "$LOCAL" = "FALSE" ]; then
  echo "Pulling dev image from registry..."
  docker pull $DEV_TAG || true
fi

if [ "$MODE" = "--test" ]; then
  echo "Testing dev image..."
  docker buildx build \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    -f docker/Dockerfile.app \
    $PLATFORM_ARG \
    --target linter \
    --build-arg BUILD_FLAGS="$BUILD_FLAGS" \
    .
fi

if [ "$BUILD_APP_IMAGE" = "FALSE" ]; then
  echo "Skipping app image build"
  exit 0
fi

# --- APP (rover) image ---
echo "Starting rover image (base: $BASE_TAG)"
docker buildx build \
  --build-arg BASE_IMAGE=$BASE_TAG \
  --build-arg DEV_IMAGE=$DEV_TAG \
  -f docker/Dockerfile.app \
  --target rover \
  $PLATFORM_ARG \
  -t $APP_TAG \
  -t $APP_SHA_TAG \
  --build-arg BUILD_FLAGS="$BUILD_FLAGS" \
  $MODE \
  .
