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

开启调试模式后，会在控制台输出识别结果等信息，并且会在窗口中显示图像识别结果。

## 运行
直接运行 `armor_detect` 即可。

### 参数设置
参数通过环境变量来设置：
* `RM_WIDTH`、`RM_HEIGHT`：摄像头使用的分辨率，`RM_WIDTH` 默认为 `640`，`RM_HEIGHT` 默认为 `480`。
* `RM_ENEMY_COLOR`：默认的敌方颜色，`blue` 或 `red`。默认值是 `blue`。
