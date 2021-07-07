# [Open-Source LiDAR Time Synchronization System by Mimicking GPS-clock](https://arxiv.org/abs/2107.02625)<sup>1</sup>

To clone the repository with proper submodules clone with `--recurse-submodules` argument:  
`git clone --recurse-submodules https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps.git`

## HW
<table>
  <tr> <td>Lidar</td> <td>Velodyne VLP-16</td> </tr>
  <tr> <td>MCU-platform</td> <td>STM32F4DISCOVERY</td> </tr>
  <tr> <td>IMU</td> <td>MPU-9150</td> </tr>
</table>

The STM32 MCU-platform is chosen as it meets all the requirements described in the paper.

## FW
The MCU firmware is designed using STM32CubeIDE for Linux. The MCU firmware can be used independently on ROS due to plain ASCII strings that may parsed by another reading API.

## SW
The software consists of ROS drivers for handling Lidar and IMU data and precise timestamping.
Lidar ROS driver is based on common ROS package with our patch for hardware timestamping by mimicking GPS-clock.  
IMU ROS driver is developed from scratch and produces [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) messages.


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
