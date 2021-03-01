# RoboRTS NWPU variant
This repository includes code from:
* https://github.com/RoboMaster/RoboRTS
* https://github.com/nwpu-v5-team/ICRA-RoboMaster-2020-Planning
* https://github.com/nwpu-v5-team/ICRA-RoboMaster-2020-Strategy
* https://github.com/nwpu-v5-team/ICRA-Firefly-Emulator

## Preparations
```
sudo apt install ros-melodic-libg2o libgoogle-glog-dev ros-melodic-joy ros-melodic-map-server ros-melodic-amcl ros-melodic-move-base ros-melodic-ros-control ros-melodic-ros-controllers
# restart shell
mkdir -p ~/.gazebo/models && cp -r src/RoboRTS/icra_robomaster_emulator/wall-2020/icra_ground_plane ~/.gazebo/models/icra_ground_plane
```
