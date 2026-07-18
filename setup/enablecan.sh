#!/bin/bash

# Bring down link before configuring
ip link set can0 down
ip link set can1 down

# Set pinmux
busybox devmem 0x0c303000 32 0x0000C400 #can1_dout
busybox devmem 0x0c303008 32 0x0000C458 #can1_din
busybox devmem 0x0c303010 32 0x0000C400 #can0_doutM
busybox devmem 0x0c303018 32 0x0000C458 #can0_din

# Load kernel drivers
modprobe can
modprobe can_raw
modprobe mttcan

# Configure and bring up the link
ip link set can0 type can bitrate 1000000
ip link set can1 type can bitrate 1000000
ip link set up can0
ip link set up can1

exit 0
