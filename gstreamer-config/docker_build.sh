#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="cprtsoftware/rover-gstreamer"
GIT_SHA=$(git rev-parse HEAD)
TAG="$IMAGE_NAME:latest"


# Default settings
MODE="--load"
OVERRIDE_ARCH=""

# Change to script directory
cd "$(dirname "$0")"

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
    *)
      echo "Usage: $0 [--push|--load (default)] [--arch <amd64|arm64> (optional for emulation)]"
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
  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
else
  docker buildx use "$BUILDER_NAME"
fi

# Define tags

if [ "$MODE" == "--push" ]; then
  echo "Push detected: building multi-arch gstreamer image"
  PLATFORMS="linux/amd64,linux/arm64"
else
  PLATFORMS=$TARGETARCH
fi

# --- Gstreamer image ---
docker buildx build \
  -f Dockerfile \
  -t cprtsoftware/rover-gstreamer:latest \
  -t cprtsoftware/rover-gstreamer:$GIT_SHA \
  --platform $PLATFORMS \
  --cache-from type=registry,ref=cprtsoftware/rover-gstreamer:cache \
  --cache-to type=registry,ref=cprtsoftware/rover-gstreamer:cache,mode=max \
  $MODE \
  .
