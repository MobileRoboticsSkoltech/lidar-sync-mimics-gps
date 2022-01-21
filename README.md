# [Open-Source LiDAR Time Synchronization System by Mimicking GPS-clock](https://arxiv.org/abs/2107.02625)<sup>1</sup>

___The LiDAR-to-IMU time synchronized system mimicing a GPS-supplied clock interface by a microcontroller-powered platform that provides 1 microsecond synchronization precision.___

<p align="center">
  <img src="https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps/blob/main/block_scheme.png">
</p>

To clone the repository with proper submodules utilize `--recurse-submodules` argument:  

`git clone --recurse-submodules https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps.git`

## Hardware
<table>
  <tr> <td>Lidar</td> <td>Velodyne VLP-16</td> </tr>
  <tr> <td>MCU-platform</td> <td>STM32F4DISCOVERY</td> </tr>
  <tr> <td>IMU</td> <td>MPU-9150</td> </tr>
</table>

The STM32 MCU-platform is chosen as it meets all the requirements described in the paper. The IMU is fed by external MCU reference clock for data rate stability.

## Firmware
The MCU firmware aimed on IMU-data gathering and LiDAR-to-IMU time synchronization. It is designed using STM32CubeIDE for Linux. 

The MCU outputs IMU data as are plain ASCII strings that can be used independently on ROS and may be parsed by another data recording/processing API. These strings contain timestamp (minutes, seconds, subseconds (1/25600000 of second) and IMU data (3D-angular velocity, temperature, 3D-acceleration) with format:  
`"i0%02x %02x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n"`

## Software
The software consists of ROS drivers for handling Lidar and IMU data and precise timestamping.
Lidar ROS driver is based on common ROS package with our patch for hardware timestamping by mimicking GPS-clock.  
IMU ROS driver is developed from scratch and produces [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) messages.

## In case of questions
Any question — raise an issue, please.

<sup>1</sup> :

```
@misc{faizullin2021opensource,
  title={Open-Source LiDAR Time Synchronization System by Mimicking GPS-clock}, 
  author={Marsel Faizullin and Anastasiia Kornilova and Gonzalo Ferrer},
  year={2021},
  eprint={2107.02625},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```
