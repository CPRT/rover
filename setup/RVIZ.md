# Instructions to use rviz2 inside the Docker Container

On my Jetson Orin Nano, I have wayland by default and not X11. To use rviz2 we need X11.

## X11
```
sudo apt update
sudo apt install xorg openbox
```

Make sure wayland is installed: sudo apt install xwayland



Maybe need this inside container:
sudo apt-get install qt5-qmake qtbase5-dev qtchooser qtbase5-dev-tools libxcb1 libx11-xcb1


Probably need this:
sudo apt-get install libx11-dev libxext-dev libxi-dev libxrandr-dev libxinerama-dev libxcomposite-dev libxcursor-dev



## Checking if X11 is running and temporarily starting it if not
If this returns anything then X11 is started and skip to next heading
```
ps aux | grep Xorg
```

If above returns nothing
```
startx
```



### Might need this
ls -s $XAUTHORITY ~/.Xauthority && chmod 644 ~/.Xauthority 
