# armor_detector
装甲板识别代码

## 编译
```
cmake .
make
```
会在当前目录下生成可执行文件 `armor_detect`。

### 调试模式
在 [`src/main.cpp`](https://github.com/njtech-robomaster/armor_detector/blob/master/src/main.cpp#L10) 中定义了 `DEBUG` 宏，如果不需要开启调试模式，则可以将该行注释。

开启调试模式后，会在控制台输出识别结果、串口收发数据等信息，并且会在窗口中显示图像识别结果。

## 运行
直接运行 `armor_detect` 即可。

### 参数设置
参数通过环境变量来设置：
* `RM_SERIAL`：与下位机通信的串口，默认为 `/dev/null`。串口通信的详细介绍参考[串口通信格式](#串口通信格式)。
* `RM_CAMERA`：使用的摄像头，默认为 `0`。
* `RM_WIDTH`、`RM_HEIGHT`：摄像头使用的分辨率，`RM_WIDTH` 默认为 `640`，`RM_HEIGHT` 默认为 `480`。
* `RM_POSE_PARAM`：姿态估计所使用的摄像头内参文件路径，如不指定则不会进行姿态估计。
  * 如指定的是一个文件，则直接从该文件中读取参数。文件格式参考[相机内参文件格式](#相机内参文件格式)。
  * 如指定的是一个目录，则从该目录下的 `{width}x{height}.xml` 文件中读取参数（`{width}` 和 `{height}` 为摄像头的分辨率）。
    * 如 `RM_POSE_PARAM=params`，并且摄像头分辨率为 640x480，则会从 `params/640x480.xml` 中读取参数。

## 串口通信格式
串口波特率为 115200。

### 上位机 -> 下位机

#### 装甲板识别结果
程序每处理一帧图像就会发送一个以下的数据包：
```
uint8 packet_header = 0x22
int16_be center_x // 目标在图像中的 x 相对坐标；-1 代表未识别到，[0, 32767] 之间的数代表 x 坐标（0 为最左侧，32767 为最右侧）
int16_be center_y // 同上，目标在图像中的 y 相对坐标
int16_be t_x // 目标在摄像头参考系中的三维 x 坐标（mm）
int16_be t_y // 同上，目标在摄像头参考系中的三维 y 坐标（mm）
int16_be t_z // 同上，目标在摄像头参考系中的三维 z 坐标（mm）
int16_be r_x // 未使用
int16_be r_y // 未使用
int16_be r_z // 未使用
uint8 packet_end = 0x33
```
如果识别到了多个装甲板目标，则只会选择其中一个发送给下位机。

## 相机内参文件格式
```xml
<opencv_storage>
	<camera_intrinsic type_id="opencv-matrix">
		<rows>3</rows>
		<cols>3</cols>
		<dt>d</dt>
		<data>
			{f_x} 0 {c_x}
			0 {f_y} {c_y}
			0 0 1
		</data>
	</camera_intrinsic>
	<distortion_coefficients type_id="opencv-matrix">
		<rows>5</rows>
		<cols>1</cols>
		<dt>d</dt>
		<data> <!-- k_1, k_2, k_3 为径向畸变系数; p_1, p_2 为切向畸变系数 -->
			{k_1}
			{k_2}
			{p_1}
			{p_2}
			{k_3}
		</data>
	</distortion_coefficients>
</opencv_storage>
```
