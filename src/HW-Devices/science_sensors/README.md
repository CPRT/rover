# Ramen

Turn on ramen node

```
sudo -E bash -c '                                                                                                   
  source /opt/ros/humble/setup.bash
  source ~/cprt_rover_24/install/setup.bash
  ros2 service call /get_raman_spectrum interfaces/srv/Raman \
    "{inittime: 5000, scansavg: 1, smoothing: 1}"
'
```

Call Node

```
sudo -E bash -c '                                                                                                   
  source /opt/ros/humble/setup.bash
  source ~/cprt_rover_24/install/setup.bash
  ros2 service call /get_raman_spectrum interfaces/srv/Raman \
    "{inittime: 5000, scansavg: 1, smoothing: 1}"
'
```
```
Transfer data
scp cprt@192.168.0.55:<filename>
```