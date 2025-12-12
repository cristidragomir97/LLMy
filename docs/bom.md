# LLMy Bill of Materials (BOM)

This document provides a comprehensive breakdown of all components needed to build LLMy, with pricing and sourcing information for both US and EU markets.

#### 🤖 Main Platform
| Part | Amount | Unit Cost (US) | Buy (US) | Unit Cost (EU) | Buy (EU) | Total(US) | Total(EU) |
|:---|:---:|:---:|:---|:---:|---:|:-:|-|
| 12V ST3215 Feetech Servo | 12 | $13.89 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-STS3215-SO-ARM100-Servo-12V_1601292634404.html) | €13.38 | [Alibaba EU](https://www.alibaba.com/product-detail/Feetech-STS3215-SO-ARM100-Servo-12V_1601292634404.html) | $166.68 | €160.56 |
| WaveShare ESP32 Servo Controller | 1 | $10.55 | [Amazon](https://www.amazon.com/Expansion-Bluetooth-Control-Application-Structures/dp/B0CKT8BN73) | €4.76 | [Kamami](https://kamami.pl/en/Servo-Controllers/1187948-serial-bus-servo-driver-board-integrates-servo-power-supply-and-control-circuit-applicable-for-st-5906623428014.html) | $10.55 | €4.76 |
| Adafruit USB-PD Trigger | 2 | $5.95 | [Adafruit](https://www.adafruit.com/product/5807) | €6.90 | [Kamami](https://kamami.pl/en/wyzwalacze-usb-pd-elektonika/1188206-usb-type-c-power-delivery-dummy-breakout-module-with-usb-type-c-power-delivery-husb238-power-supply-controller-5906623469918.html) | $11.90 | €13.80 |
| Adafruit ICM20948 IMU | 1 | $14.95 | [Adafruit](https://www.adafruit.com/product/4554) | €18.91 | [Kamami](https://kamami.pl/en/sensors-6dof-9dof-10dof/587247-stemma-qt-tdk-invensense-icm-20948-9-dof-imu-module-with-9-dof-sensor-icm-20948-adafruit-4554-5906623430239.html) | $14.95 | €18.91 |
| Baseus GP12 Battery Pack (145W, 20.8Ah) | 1 | $59.99 | [Amazon](https://www.amazon.com/Baseus-20800mAh-Portable-Charger-Charging/dp/B0DQTYLGYK) | €48.99 | [Baseus EU](https://eu.baseus.com/products/energeek-gp12-power-bank-145w-20800mah) | $59.99 | €48.99 |
| **Total Base Platform** | | | | | | **$264.07** | **€247.02** |

#### 📷 RGB-D Cameras

**The core perception system** for LLMy uses RGB-D cameras that provide both color and depth information. This enables rich scene understanding for AI inference, manipulation, and navigation.

> 💡 **Why RGB-D?** A single sensor provides object identification with distance/size estimation. The depth stream supports multiple SLAM approaches: RGB-SLAM, RGB-D SLAM, or 2D SLAM via `depthimage_to_laserscan`.

All software is built on the **Intel RealSense SDK** and ROS driver, but other depth cameras serve as near drop-in replacements.


| Part                | Unit Cost (US) | Buy (US) | Unit Cost (EU) | Buy (EU) | Robot + Camera (US) | Robot + Camera (EU) |
|---------------------|---------------:|:---------|---------------:|:---------|-----------------------------:|-----------------------------:|
| **YDLidar HP60C** | $202.00 | [RobotShop US](https://www.robotshop.com/products/ydlidar-hp60c-compact-lidar-sensor-02-4m-range-73-8-scan-angle) | €190.00 | [RobotShop EU](https://eu.robotshop.com/products/ydlidar-hp60c-compact-lidar-sensor-02-4m-range-73-8-scan-angle) | **$466.07** | **€437.02** |
| **Orbbec Gemini 2** | $234.00 | [Orbbec Store](https://store.orbbec.com/products/gemini-2) | €230.00 | [Orbbec EU](https://store.orbbec3d.com) | **$498.07** | **€477.02** |
| **Intel RealSense D415** | $272.00 | [RealSense Store](https://store.realsenseai.com/buy-intel-realsense-depth-camera-d415.html) | €310.00 | [Mouser EU](https://eu.mouser.com/ProductDetail/Intel/82635AWGDVKPRQ) | **$536.07** | **€557.02** |
| **Intel RealSense D435** | $314.00 | [RealSense Store](https://store.realsenseai.com/buy-intel-realsense-depth-camera-d435.html) | €317.60 | [Mouser EU](https://eu.mouser.com/ProductDetail/Intel/82635AWGDVKPRQ) | **$578.07** | **€564.62** |
| **Stereolabs ZED 2i** | $499.00 | [Stereolabs Store](https://store.stereolabs.com/products/zed-2i) | €449.00 | [Stereolabs EU](https://store.stereolabs.com/products/zed-2i) | **$763.07** | **€696.02** |



#### 💻 Single Board Computers & Mini-PCs

**Recommended:** Nvidia Orin Nano Super for optimal AI performance. The battery pack delivers up to **65W per power lane** at **20V**, supporting a wide range of compute options:

| Option | Power Draw | Performance | Best For |
|--------|------------|-------------|----------|
| 🥧 **Raspberry Pi 5** | ~15W | Basic | Learning, simple tasks |
| 🚀 **Nvidia Orin Nano** | ~25W | 67 TOPS of AI performace  | AI/ML inference |
| 💪 **Intel NUC** | ~30-45W | Better, Desktop Class CPU| General compute, ROS2 |
| 🔧 **Custom Mini-PC** | <65W | Variable | Specific requirements |

> 💡 **Power Note:** Any device requiring ≤65W and compatible with 20V USB-C PD will work.


## 🔧 Optional Components

### 📡 LIDAR Sensor

<div align="center">
  <img src="https://static.generation-robots.com/img/image-3-rplidar-c1.jpg" alt="RPLidar C1" width="300"/>
</div>

While RGB-D cameras provide 2D scan data via `depthimage_to_laserscan`, they typically have a 30cm minimum range blind spot. A dedicated LIDAR sensor offers:

- **Enhanced Navigation** - Better obstacle avoidance and SLAM
- **Close-Range Detection** - No minimum distance limitations
- **Simpler Processing** - Direct 2D scan data without conversion
- **Reliability** - Consistent performance in various lighting conditions

#### Recommended LIDAR Options

| Model | Range | Price (US) | Price (EU) | Buy |
|-------|-------|------------|------------|---------|
| **SLAMTEC RPLidar C1** | 0.1-12m | $89 | €79 | [Kamami EU](https://kamami.pl/en/laser-scanner/1189121-slamtec-rplidar-c1-laser-ranging-sensor-360-omnidirectional-lidar-millimeter-level-high-definitio-5906623483501.html) |
| **YDLIDAR X2L** | 0.12-8m | $99 | €89 | [RobotShop](https://www.robotshop.com/products/ydlidar-x2l-lidar-scanner) |

> 💡 **Installation:** LIDAR mounts on the camera tower with provided brackets.

