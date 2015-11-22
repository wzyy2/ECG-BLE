# ECG-BLE
This repository contains hardware designs and software for my graduate project.<br>
Any issues or improvements please contact jacob-chen@iotwrt.com.

## Introduction
### 设计
TI CC2640 + ADS1191 心电前端 + ADXL335 三轴传感器

### 功能
- 胸导联心电监测
- 跌倒检测

### 外观

## About App交接

### 搜索、连接
名称：`ECG-CH` <br>
//广播格式：<br>
以上主要用于搜索 连接设备，自行选择<br>
不需要配对

### 服务
只有一个Service
Heartrate Service `uuid：0x180D`

| Characteristic Name  | UUID | Mandatory Properties   |  Security Permissions |
|  :----:  |:----: |  :----:  | :----:  |
| Heart rate measurement  | 0x2A37 |  Notify | None   |
| Fall-Down      |  0x2A39   | Read,Write |   None   |

`Heart rate measurement`要开启Notify的话需要向他的`Client Characteristic Configuration uuid：0x2902`写一,关闭写零，大致40ms发一到两个包，每个包有2-20个字节，两个相邻字节组成16位有符号数据，低位先发。
跌倒检测的话app就定时读一下值吧(注意取下，放置装置等动作很容易被识别为摔倒），值一为已摔倒，值零为检测中，检测到摔倒以后需要写零，重新检测。

### 信号处理
由于mcu端为了节省功耗，没有做任何处理，大致对输出的信号还需要`去直流分量`，`工频滤波`，`呼吸滤波`等等。

### 参考App例程
参考例程基于qt5.5的程序改造，可以编译在Android和iOS上，可以蓝牙连接，显示波形，并做了DC Removal以及50Hz Notch，可以参考参考。


