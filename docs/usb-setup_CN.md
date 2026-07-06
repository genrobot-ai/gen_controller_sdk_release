# USB 配置指南

本文说明如何配置 udev 规则，使每个 Gen Controller 的 USB 口映射到固定的串口和相机设备名。

模板文件位于 [config/99-usb-serial.rules](../config/99-usb-serial.rules)。

配置完成后，对应 USB 口可识别 Gen Controller 设备，后续无需再次配置。

[English](usb-setup.md)

## 1 单爪 USB 口配置

最终配置形式如图。

![单爪 udev 配置示例](../image/image_1.png)

用户需修改的地方如下。

![需修改字段](../image/image_2.png)

### 1.1 参数 1 — 串口 KERNELS

```shell
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB* | grep -E "KERNELS|DRIVERS"
```

如果获取不到串口，执行：

```shell
sudo apt remove brltty
```

将输出中的**第二个** `KERNELS` 数值配置到参数 1 处：

![串口 KERNELS 示例](../image/image_3.png)

### 1.2 参数 2 — 相机 KERNELS

```shell
v4l2-ctl --list-devices
```

输出示例：

![v4l2 设备列表](../image/image_4.png)

针对该 USB 的第一个相机执行：

```shell
udevadm info -a -n /dev/video* | grep -E "KERNELS|SUBSYSTEMS"
```

将输出中的**第一个** `KERNELS` 数值配置到参数 2 处：

![相机 KERNELS 示例](../image/image_5.png)

### 1.3 加载配置

```shell
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 2 双爪 USB 口配置

最终配置形式如图所示。

![双爪 udev 配置示例](../image/image_6.png)

需修改地方：

![双爪需修改字段](../image/image_7.png)

步骤：

1. 插入**左**夹爪，按单爪方法配置；
2. 拔下左夹爪，插入**右**夹爪，再次按单爪方法配置；
3. 加载 udev 规则。

双爪配置后的默认串口软链接：

| 设备 | 软链接 |
|------|--------|
| 左夹爪 | `/dev/ttyDeviceLeft` |
| 右夹爪 | `/dev/ttyDeviceRight` |

## 3 多爪配置

在 `99-usb-serial.rules` 中按相同方式为每个夹爪添加条目。

## 4 验证设备 ID（可选）

在 `roscore` 运行状态下：

```shell
cd src/robot_driver/scripts/
bash camera_cmd.sh MCUID
```

示例输出：

![设备 ID 示例](../image/image_8.jpg)
