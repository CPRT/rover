#!/bin/bash

sudo apt-get update
sudo apt-get install -y libusb-1.0-0-dev git build-essential

cd "$(dirname "$0")"

sudo cp reset_usb.service /etc/systemd/system/reset_usb.service
sudo chmod 644 /etc/systemd/system/reset_usb.service

cd /tmp
git clone https://github.com/mvp/uhubctl.git
cd uhubctl
make
sudo make install

sudo systemctl daemon-reload
sudo systemctl enable reset_usb.service
sudo systemctl start reset_usb.service
