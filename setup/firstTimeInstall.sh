#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
CURRENT_DIR="$(pwd)"

if [ "$SCRIPT_DIR" == "$CURRENT_DIR" ] || [ "$SCRIPT_DIR" == "." ]; then
    echo "The script is being run from its own directory."
else
    echo "The script is NOT being run from its own directory."
    exit 1
fi

source ~/.bashrc
./setup_service.sh

# Jetson Only setup
if [ -f /etc/nv_tegra_release ]; then
    echo "Jetson detected, running Jetson specific setup."
else
    echo "Non-Jetson system detected, skipping Jetson specific setup."
    exit 0
fi

sudo cp enablecan.sh /usr/local/bin || exit 1
echo "$USER ALL=(ALL) NOPASSWD: /usr/local/bin/enablecan.sh" | sudo tee -a /etc/sudoers > /dev/null

./usb_service_setup.sh

UDEV_RULE='SUBSYSTEM=="video4linux", ATTR{name}=="Arducam B0495 (USB3 2.3MP)", ATTRS{idVendor}=="04b4", ATTR{index}=="0", ATTRS{idProduct}=="0495", SYMLINK+="v4l/by-id/drive_camera", MODE="0666"'
echo "$UDEV_RULE" | sudo tee /etc/udev/rules.d/99-arducam.rules > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

# Set up Zed SDK
# Needed on host because of https://gist.github.com/adujardin/2d5ce8f000fc6a7bd40bee2709749ff8
# As good practice use the version that matches the docker one
cd /tmp
curl -L https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.4/ZED_SDK_Tegra_L4T36.4_v5.4.0.zstd.run -o zed_sdk_installer.run
chmod +x zed_sdk_installer.run
./zed_sdk_installer.run -- silent