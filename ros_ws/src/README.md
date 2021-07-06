# Samsung Avatar Dataset

## Sensors utilized:
<table>
    <tr> <td>Lidar</td> <td>Velodyne VLP-16</td> </tr>
    <tr> <td>MCU-platform</td> <td>STM32F4DISCOVERY</td> </tr>
    <tr> <td>IMU</td> <td>MPU9150</td> </tr>
</table>

## Prerequisites
- OS version: Ubuntu 18.04  
- ROS version: Melodic. Install Desktop-Full from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

- Velodyne related notes
    - To set up lidar go to [tutorial](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) and do __only__ step `1.1 Configure your computer’s IP address through the Gnome interface` and just in case `4. Viewing the Data`

    - In case of problem `fatal error: pcap.h: No such file or directory` do  
`sudo apt-get install -y libpcap-dev`  
- To be able to use serial interface by MCU package, run  
`sudo usermod -a -G dialout $USER`
Then log out and log in.  

    
## Full roslaunch
The following command is used for launching all of the sensors:  

`roslaunch data_collection data_collection.launch`

## Velodyne LIDAR
Patched package is used for hardware time synchronization of lidar


## MCU-IMU node
MCU-IMU node is receiving data from hardware platform via virtual serial port and publishing the following topics:  
`/imu` - IMU measurements  
`/imu_temp` - IMU measured temperature  
`/lidar_ts` - platform local timestamps of triggered pulses for hardware lidar synchronization  

## Serial
`serial` is a C++ library for seial interface with MCU ([repo](https://github.com/wjwwood/serial.git)).
