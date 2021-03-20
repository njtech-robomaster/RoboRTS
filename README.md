# armor_detector
RealSense 装甲板识别包

## 依赖
此包依赖 librealsense 和 opencv 4。

依赖安装命令：
```
sudo apt install librealsense2-dev librealsense2-dkms libopencv-dev
```

## ROS API
此包含有一个节点 `armor_detector`，其使用 RealSense 进行装甲板识别并发布结果。

### 参数
* `~color_resolution_width`, `~color_resolution_height`
  * RGB 摄像头所使用的分辨率，默认 1920x1080。
* `~depth_resolution_width`, `~depth_resolution_height`
  * 深度摄像头所使用的分辨率，默认 1280x720。
* `~camera_frame`
  * 摄像头的 TF Frame 名称，默认 `camera_link`。
  * 该坐标系为视觉坐标系，即相对于摄像头：x+ 向右，y+ 向下，z+ 向前。
* `team_color`
  * 己方队伍颜色，`red` 或 `blue`。`armor_detector` 会将另一种颜色的装甲板作为识别对象。

### 发布的主题
* `armor_target` (`geometry_msgs/PointStamped`)
  * 识别到的装甲板的位置。
