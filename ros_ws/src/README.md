# Samsung Avatar Dataset

## Sensors utilized:
<table>
    <tr> <td>Lidar</td> <td>Velodyne VLP-16</td> </tr>
    <tr> <td>Visual cameras</td> <td>Basler</td> </tr>
    <tr> <td>Depth camera</td> <td>Kinect Azure</td> </tr>
    <tr> <td>MCU-platform</td> <td>STM32F4DISCOVERY</td> </tr>
    <tr> <td>IMU</td> <td>MPU9150</td> </tr>
    <tr> <td>Phone</td> <td>Samsung S10E</td> </tr>
</table>

## Prerequisites
- OS version: Ubuntu 18.04  
- ROS version: Melodic. Install Desktop-Full from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

- Before building the packages by `catkin_make` command the following software must be installed:
    - for Basler visual cameras __pylon 5.1.0 Camera Software Suite Linux x86 (64 bit) - Debian Installer Package__ from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/pylon-5-1-0-linux-x86-64-bit-debian/)  
    - for Azure camera __Azure Kinect Sensor SDK__ from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions)  
      - to run `k4aviewer` or `k4arecorder` of the SDK without root follow [this](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup)
- Velodyne related notes
    - Velodyne package must be clonned from patched branch by:  
`git clone --single-branch --branch mrob-patches https://github.com/anastasiia-kornilova/velodyne.git`  

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

## Basler Visual Cameras
You might need some dependencies (not if install full version)
```
sudo apt install ros-melodic-camera-info-manager ros-melodic-image-geometry
```

## Azure Depth Camera


## MCU-IMU node
MCU-IMU node is receiving data from hardware platform via virtual serial port and publishing the following topics:  
`/imu` - IMU measurements  
`/imu_temp` - IMU measured temperature  
`/lidar_ts` - platform local timestamps of triggered pulses for hardware lidar synchronization  
`/cameras_ts` - platform local timestamps of triggered pulses for hardware visual and depth cameras synchronization  

## Serial
`serial` is a C++ library for seial interface with MCU ([repo](https://github.com/wjwwood/serial.git)).

## Samsung S10E Phone


## Useful commands
- save images from current publishing image topic `/pylon_camera_node0/image_raw` to `png` images in current directory
`rosrun image_view image_saver image:=/pylon_camera_node0/image_raw _filename_format:=%04i.png _encoding:=rgb8`  

## Network setup
Bandeja laptop is at 192.168.1.2, same as all computer laptops describes in other robot configurations (see akula-driver)
To solve any issue with fingerprinting, you can disable it by editing the file `~./ssh/config`
```
Host 192.168.1.*
   StrictHostKeyChecking no
   UserKnownHostsFile=/dev/null
```
and adding to your table `/etc/hosts` the proper ip-name
