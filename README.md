# RoboRTS NWPU variant
This repository includes code from:
* https://github.com/RoboMaster/RoboRTS
* https://github.com/nwpu-v5-team/ICRA-RoboMaster-2020-Planning
* https://github.com/nwpu-v5-team/ICRA-RoboMaster-2020-Strategy
* https://github.com/nwpu-v5-team/ICRA-Firefly-Emulator

## Preparations
```
sudo apt install ros-melodic-libg2o libgoogle-glog-dev ros-melodic-joy ros-melodic-map-server ros-melodic-amcl ros-melodic-move-base ros-melodic-ros-control ros-melodic-ros-controllers liboctomap-dev
git clone https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin.git src/gazebo_ros_2Dmap_plugin
mkdir -p ~/.gazebo/models && ln -sfT $(realpath src/RoboRTS/icra_robomaster_emulator/wall-2020/icra_ground_plane) ~/.gazebo/models/icra_ground_plane
```

## Generate perfect map
```
roslaunch icra_robomaster_emulator generate_perfect_map.launch
```
After the command finishes, `map.yaml` and `map.pgm` are saved to the current directory.
