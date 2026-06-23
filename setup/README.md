# Setup

## firstTimeInstall.sh
Runs all the setup scripts. Only for use on the Jetson after flashing.

## setup_service.sh (Rover Only)
Sets up a linux service that starts the rover launcher that is connected to via our webUI

## start_rover.service
The linux service file that starts the core rover elements

## start_rover.sh
Script that starts core rover elements

## enablecan.sh
Script that must be run on startup to enable the can interface

## draw_hitboxes.py
Python app that helps you turn stls into a series of simple shapes to insert into the URDF as collision boxes

## reset_usb.service/usb_service_setup.sh
The service utils to setup and run the usb reset on startup. This gets around our drive camera needing to be re-plugged after the jetson is on

## v4l2_settings_controller
A GUI to control brightness of the cameras. This should be integrated into the ros2 ecosystem