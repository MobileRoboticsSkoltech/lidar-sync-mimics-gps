# [Open-Source LiDAR Time Synchronization System by Mimicking GPS-clock](https://arxiv.org/abs/2107.02625)<sup>1</sup>

___The LiDAR-to-IMU time synchronized system mimicing a GPS-supplied clock interface by a microcontroller-powered platform that provides 1 microsecond synchronization precision.___

<p align="center">
  <img src="https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps/blob/main/block_scheme.png">
</p>

To clone the repository with proper submodules utilize `--recurse-submodules` argument:  

`git clone --recurse-submodules https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps.git`

## System requirements
The project was tested using:
- Ubuntu 18.06
- ROS Melodic Morenia
- STM32CubeIDE for Linux

## Hardware
<table>
  <tr> <td>Lidar</td> <td>Velodyne VLP-16</td> </tr>
  <tr> <td>MCU-platform with built-in programmer</td> <td>STM32F4DISCOVERY</td> </tr>
  <tr> <td>IMU</td> <td>MPU-9150</td> </tr>
  <tr> <td>UART-to-USB stick</td> <td>any, we used stick based on CP2102 stone</td> </tr>
  <tr> <td>Signal inverter</td> <td>any, see 
<a href="#note-about-mcu-to-lidar-data-signal-inverter">Note about MCU to Lidar data signal inverter</a> below</td> </tr>
</table>

The STM32 MCU-platform is chosen as it meets all the requirements described in the paper. The IMU is fed by external MCU reference clock for data rate stability.

## Firmware
The MCU firmware aimed on IMU-data gathering and LiDAR-to-IMU time synchronization. It is designed using STM32CubeIDE for Linux. 
The main source code is placed in [main.c](https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps/blob/main/firmware/Core/Src/main.c) while the other setups are stored in [firmware.ioc](https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps/blob/main/firmware/firmware.ioc) and setup through the IDE GUI. It means there is no need to edit other source files except two mentioned.

## IMU data format
The MCU outputs IMU data as are plain ASCII strings that can be used independently on ROS and may be parsed by another data recording/processing API. These strings contain timestamp (minutes, seconds, subseconds (1/25600000 of second) and IMU data (3D-angular velocity, temperature, 3D-acceleration) with format:  
```
"i0%02x %02x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n"
 \/\__/ \__/ \__/ \_______/ \____________/ \__/ \____________/
 | hour mins secs  subsecs     3D gyro     temp     3D acc
IMU0 identifier for compatibility with other projects
```

## Software
The software consists of ROS drivers for handling Lidar and IMU data and precise timestamping.
Lidar ROS driver is based on common ROS package with our patch for hardware timestamping by mimicking GPS-clock.  
IMU ROS driver is developed from scratch and produces [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) messages.

## In case of questions
Any question — raise an issue, please.

## Wiring

<table>
  <tr> <td>MCU pin</td> <td>Name</td> <td>Role</td> </tr>
  <tr> <td>PB6</td> <td>I2C1_SCL</td> <td>I2C SCL line for IMU module data transfer</td> </tr>
  <tr> <td>PB7</td> <td>I2C1_SDA</td> <td>I2C SDA line for IMU module data transfer</td> </tr>
  <tr> <td>PH0</td> <td>RCC_OSC_IN</td> <td>Crystal resonator connection, already connected to MCU by default</td> </tr>
  <tr> <td>PH1</td> <td>RCC_OSC_OUT</td> <td>Crystal resonator connection, already connected to MCU by default</td> </tr>
  <tr> <td>PA0</td> <td>TIM5_CH1</td> <td>Output compare signal, Not used</td> </tr>
  <tr> <td>PA6</td> <td>TIM3_CH1</td> <td>Reference clock source for IMU</td> </tr>
  <tr> <td>PE9</td> <td>TIM1_CH1</td> <td>PPS signal output</td> </tr>
  <tr> <td>PD12</td> <td>GPIO_OUT</td> <td>Debugging LED. IMU interrupt came</td> </tr>
  <tr> <td>PD13</td> <td>GPIO_OUT</td> <td>Debugging LED. Toggling in the main loop (`while`)</td> </tr>
  <tr> <td>PD14</td> <td>GPIO_OUT</td> <td>Debugging LED. Not used</td> </tr>
  <tr> <td>PD15</td> <td>GPIO_OUT</td> <td>Debugging LED. Not used</td> </tr>
  <tr> <td>PC9</td> <td>GPIO_EXTI9</td> <td>Interrupt input pin from IMU module. IMU trigger this pin when new data sample is ready</td> </tr>
  <tr> <td>PC10</td> <td>UART4_TX</td> <td>UART Transmit line to PC through UART-to-USB stick</td> </tr>
  <tr> <td>PC11</td> <td>UART4_RX</td> <td>UART Receive line from PC through UART-to-USB stick, not used</td> </tr>
  <tr> <td>PC12</td> <td>UART5_RX</td> <td>UART Receive line from Lidar, not used</td> </tr>
  <tr> <td>PD2</td> <td>UART5_TX</td> <td>UART Transmit MCU clock line to Lidar through signal inverter (see note below)</td> </tr>
</table>

TIM5 (RTC) and TIM1(PPS signal generator, NMEA message transmission trigger) now share three tasks explained in the article. For optimization, only single timer can be utilized for all this tasks. However, to keep compatibility with current uasge of the firmware in ongoing projects, we do not plan to update it.

## Note about MCU to Lidar data signal inverter
Page 43 of section 7.4.3 Timing and Polarity Requirements [VLP-16 User Manual](https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf) states the following:

<i>The serial connection for the NMEA message follows the RS232 standard. The interface is capable of handling voltages between ±15 VDC.  
- Low voltages are marks and represent a logical 1.
- High voltages are spaces and represent a logical 0.

The serial line idle state (MARK) is a low voltage indicating a logical 1. When the start bit is asserted, the positive voltage will be asserted representing a logical 0.</i>

This means that the polarity of the input UART signal must be inverse: "1" is "Low", "0" is "High". For that case some UART transmitters can provide such a signal polarity inversion. However, UART transmitter of STM32F4 MCU cannot. To solve the issue, separate signal invertor can be utilized that simply converts UART signal to the inverse one:
```
HIGH---    ┌─────────┐    ┌───────┐                   ───┐         ┌────┐       ┌──────
           │         │    │       │         ---->        │         │    │       │
LOW---- ───┘         └────┘       └──────                └─────────┘    └───────┘
```
Single Schmitt-trigger inverter [SN74LVC1G14](https://www.ti.com/product/SN74LVC1G14) is a good choice for that purpose, we used it in a [board](https://www.chipdip.ru/product/rdc2-0015a).  
Googling of _schmitt inverter_, _74HC14_ can help.

## WIP
- adding connecting diagram

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
