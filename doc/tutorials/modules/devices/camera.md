相机设备 {#tutorial_modules_camera}
============

@author 赵曦
@date 2023/03/15

@prev_tutorial{tutorial_modules_ort}

@tableofcontents

------

相关类 rm::MvVideoCapture, rm::HikVideoCapture.

## 1. 如何使用

使用前需安装相机驱动，详情参考：@ref tutorial_build

### 1.1 初始化

@add_toggle{MindVision}

创建 MvVideoCapture 对象即可初始化相机，例如：

```cpp
MvVideoCapture capture1(GRAB_CONTINUOUS, RETRIEVE_SDK, "0123456789");
MvVideoCapture capture2(GRAB_SOFTWARE, RETRIEVE_CV, "0123456789");
```

@end_toggle

@add_toggle{HikRobot}

创建 HikVideoCapture 对象即可初始化相机，例如：

```cpp
HikVideoCapture capture1(GRAB_CONTINUOUS, RETRIEVE_SDK, "0123456789");
HikVideoCapture capture2(GRAB_SOFTWARE, RETRIEVE_CV, "0123456789");
```

@end_toggle

第 1 个参数为 **相机采集模式** ，见下表

| 采集方式 |   枚举标识符    |                         含义                         |
| :------: | :-------------: | ---------------------------------------------------- |
| 连续采集 | GRAB_CONTINUOUS | 连续触发相机，每次 `grab` 方法被执行后，均会触发相机的采样功能，一般调用 `read` 方法可间接调用 `grab`。 |
|  软触发  |  GRAB_SOFTWARE  | 软件触发，需要手动设置触发帧，`grab` 方法只有在被设置触发帧之后的下一次执行有效，否则会阻塞以等待接收到触发帧，在 `1s` 内如果没有收到触发帧，则会被认为 `grab` 失败。 |
|  硬触发  |  GRAB_HARDWARE  | 硬件触发，通过向相机的航空接头传输高低电平信号来设置触发帧，信号的有效性可通过软件设置，例如设置高电平、低电平、上升边沿、下降边沿的一种为触发方式，`grab` 方法同软触发。 |

详细的触发方式 @see

- <a href="https://vision.scutbot.cn/HikRobot/index.html" target="_blank"> HikRobot 工业相机手册 </a>

- <a href="https://vision.scutbot.cn/MindVision/mv.pdf" target="_blank"> MindVision 手册 </a>

第 2 个参数为 **相机处理模式** ，见下表

|         处理模式         |  枚举标识符  | 十六进制数 | 含义                                                         |
| :----------------------: | :----------: | :--------: | ------------------------------------------------------------ |
|  使用官方 SDK 进行处理   | RETRIEVE_SDK |   `0x01`   | 相机在捕获到 raw 图像后，采用 SDK 提供的函数进行处理，兼容性高，不依赖第三方库，但效率较低。 |
| 使用 `cvtColor` 进行处理 | RETRIEVE_CV  |   `0x02`   | 第三方库采用 OpenCV，使用 `cv::cvtColor` 进行转码操作，对多核 CPU 设备性能较高。 |
|   使用 `LUT` 进行处理    | RETRIEVE_LUT |   `0x10`   | Look-Up Table 单通道像素映射表，使用 `cv::LUT` 将 raw 图像像素值作映射，再进行转码操作。 |

### 1.2 光学参数设置

#### 1.2.1 曝光设置

手动/自动设置曝光

```cpp
capture.set(CAP_PROP_RM_MANUAL_EXPOSURE); // 手动曝光
capture.set(CAP_PROP_RM_AUTO_EXPOSURE);   // 自动曝光
```

设置曝光值

```cpp
capture.set(CAP_PROP_RM_EXPOSURE, 600); // 设置曝光值为 600
```

#### 1.2.2 白平衡设置

手动/自动设置白平衡

```cpp
capture.set(CAP_PROP_RM_MANUAL_WB); // 手动白平衡
capture.set(CAP_PROP_RM_AUTO_WB);   // 自动白平衡
```

设置各通道增益，并生效（在手动设置白平衡模式下有效）

```cpp
capture.set(CAP_PROP_RM_WB_RGAIN, 102); // 红色通道增益设置为 102
capture.set(CAP_PROP_RM_WB_GGAIN, 101); // 绿色通道增益设置为 101
capture.set(CAP_PROP_RM_WB_BGAIN, 100); // 蓝色通道增益设置为 100
```

#### 1.2.3 其余光学参数设置

```cpp
capture.set(CAP_PROP_RM_GAIN, 64);        // 设置模拟增益为 64
capture.set(CAP_PROP_RM_GAMMA, 80);       // 设置 Gamma 为 80
capture.set(CAP_PROP_RM_CONTRAST, 120);   // 设置对比度为 120
capture.set(CAP_PROP_RM_SATURATION, 100); // 设置饱和度为 100
capture.set(CAP_PROP_RM_SHARPNESS, 100);  // 设置锐度为 100
```

@note HikRobot 工业相机暂不支持修改 Gamma

### 1.3 处理参数设置

```cpp
// 设置硬触发采集延迟为 1000 μs，仅在硬触发模式下有效
capture.set(CAP_PROP_RM_TRIGGER_DELAY, 1000);
// 设置单次触发时的触发帧数为 5 帧，即一次触发能触发 5 帧画面，仅在触发模式下有效
capture.set(CAP_PROP_RM_TRIGGER_COUNT, 5);
// 设置单次触发时多次采集的周期为 100 μs，即一次触发信号能触发多帧画面，每帧间隔为 100 μs
capture.set(CAP_PROP_RM_TRIGGER_PERIOD, 100);
```

### 1.4 事件设置

```cpp
capture.set(CAP_ACT_RM_ONCE_WB);      // 执行一次白平衡操作，仅在手动白平衡模式下有效
capture.set(CAP_ACT_RM_SOFT_TRIGGER); // 执行一次软触发，仅在软触发模式下有效
```

## 2. para 参数加载

RMVL 提供了全局的相机参数对象: para::camera_param ，详情可参考类 para::CameraParam

## 3. 示例程序

在构建 RMVL 时，需开启 `BUILD_EXAMPLES` 选项（默认开启）

```bash
cmake -DBUILD_EXAMPLES=ON ..
make -j4
cd build
```

### 3.1 单相机

单相机例程，在 build 文件夹下执行以下命令

@add_toggle{MindVision}
```bash
bin/sample_mv_mono
```
@end_toggle
@add_toggle{HikRobot}
```bash
bin/sample_hik_mono
```
@end_toggle

相机按照连续采样、`cvtColor` 处理方式运行，程序运行中，`cv::waitKey(1)` 接受到 `s` 键被按下时，可将参数保存到 `out_para.yml` 文件中。

键入一次 `Esc` 可暂停程序，按其余键可恢复。键入两次 `Esc` 可退出程序。

### 3.2 多相机

多相机例程，在 `build` 文件夹下执行以下命令

@add_toggle{MindVision}
```bash
bin/sample_mv_multi
```
@end_toggle
@add_toggle{HikRobot}
```bash
bin/sample_hik_multi
```
@end_toggle

相机按照连续采样、`cvtColor` 处理方式运行，程序会枚举所有的相机设备，并可视化的显示出来，指定一个序列号来启动某一相机。

程序运行过程中，相机参数会自动从 `out_para.yml` 中加载，若没有则会按照默认值运行。

键入一次 `Esc` 可暂停程序，按其余键可恢复。键入两次 `Esc` 可退出程序。

### 3.3 相机录屏

相机录屏例程，在 build 文件夹下执行以下命令

@add_toggle{MindVision}
```bash
bin/sample_mv_writer
```
@end_toggle
@add_toggle{HikRobot}
```bash
bin/sample_hik_writer
```
@end_toggle

相机按照连续采样、`cvtColor` 处理方式运行，`-o` 可指定输出文件名，否则默认输出到 `ts.avi`，例如

@add_toggle{MindVision}
```bash
bin/sample_mv_writer -o=aaa.avi
```
@end_toggle
@add_toggle{HikRobot}
```bash
bin/sample_hik_writer -o=aaa.avi
```
@end_toggle

程序运行过程中，相机参数会自动从 `out_para.yml` 中加载，若没有则会按照默认值运行。

### 3.4 相机标定

相机标定程序，在 `build` 文件夹下执行以下命令

@add_toggle{MindVision}
```bash
bin/sample_mv_calibration -w=<?> -h=<?> -s=<?> -d=<?> -n=<?>
```
@end_toggle
@add_toggle{HikRobot}
```bash
bin/sample_hik_calibration -w=<?> -h=<?> -s=<?> -d=<?> -n=<?>
```
@end_toggle

`<?>` 表示可调节，具体帮助可直接执行以下命令

@add_toggle{MindVision}
```bash
bin/sample_mv_calibration -help
```
@end_toggle
@add_toggle{HikRobot}
```bash
bin/sample_hik_calibration -help
```
@end_toggle

## 4. 使用 Demo

@note 下面以 MvVideoCapture 为例， HikVideoCapture 相机使用方式与之完全相同

连续采样：

```cpp
int main()
{
    MvVideoCapture capture(GRAB_CONTINUOUS, RETRIEVE_CV);
    Mat frame;
    while(capture.read(frame))
    {
        imshow("frame", frame);
        if (waitKey(1) == 27)
            break;
    }
}
```

软触发：

```cpp
int main()
{
    MvVideoCapture capture(GRAB_SOFTWARE, RETRIEVE_CV);

    bool run = true;
    thread th(
        [&run]()
        {
            while (run)
            {
                this_thread::sleep_for(chrono::milliseconds(10));
                capture.set(CAP_ACT_RM_SOFT_TRIGGER); // 触发
            }
        });

    Mat frame;
    while (capture.read(frame))
    {
        imshow("frame", frame);
        if (waitKey(1) == 27)
        {
            run = false;
            break;
        }
    }
    th.join();
}
```