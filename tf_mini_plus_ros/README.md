# TF Mini Plus ROS I2C

[//]: # (Image References)

[image1]: https://cdn.sparkfun.com//assets/parts/1/3/5/9/5/15179-TF_Mini_Plus_-_Sealed_LiDAR_Module-01.jpg "TFMiniPlus"
[image2]: https://storage.googleapis.com/vision-198622-kiwibot-packages/documentation_resources/control/speed_control.png "Response"
[image3]: https://storage.googleapis.com/vision-198622-kiwibot-packages/documentation_resources/control/speed_control.gif "Speed_control"
[image4]: https://storage.googleapis.com/vision-198622-kiwibot-packages/documentation_resources/control/PID2.png "PID"
[image5]: https://storage.googleapis.com/vision-198622-kiwibot-packages/documentation_resources/control/PID3.png "PID2"

![alt text][image1]

## Short Description

This is a package and test application of an I2C driver for the TF Mini Plus Range Sensor. It was tested on Ubuntu 16.04 Xenial for the Jetson TX2

For more information about the sensor, visit:

* https://github.com/TFmini/TFmini-Plus



## Building the package

 * Install the dependencies:
    ```bash
        apt-get install libi2c-dev
        apt-get install i2c-tools
    ```
 * Clone this project to your catkin workspace's src folder
 * Run catkin_make in the catkin workspace to build the package.


## How to run the package

### 1. Change sensor address

* After powering and connecting the sensor (or sensors) to your i2c bus, you can check the sensor address using *i2cdect* (In the following example, we are checking devices connected to the bus 1 and one device appears in the address 0x10).

    ```bash
    i2cdetect -y 1

        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- UU -- -- -- -- -- -- --
    70: -- -- -- -- -- -- -- --
    ```
* If you want to change the sensor to a new address:
    ```bash
    rosrun tf_mini_plus_ros change_address_node _address1:=16 _address2:=20 _bus:=1
    ```
    where **address1** corresponds to the original sensor address and **address2** corresponds to the new address we want to give it, while **bus** corresponds to the bus we have connected the sensor to. Please keep in mind that the addresses are entered in decimal format, even though they are usually read in hexadecimal format. Hence the address 16 is equal to 0x10 and the address 20 is equal to 0x14.

### 3. Start reading data from sensors.

* First, make sure that the parameters in the *launch/tf_mini_plus.launch* file are properly set:
  *  tfmp_base_address: Corresponds to the sensor address (or the first address if multiple sensors are connected). Keep in mind that it is to be entered in decimal format.
  *  tfmp_bus: Corresponds to the bus where the sensor or sensors are connected
  *  tfmp_num_devices: Corresponds to the number of sensors that are going to be connected to the bus
  
  Keep in mind that the sensors must have consecutive addresses. For example: if we have 3 sensors connected to the bus 0 on the addresses 0x11, 0x12 and 0x13. The parameters should be:
    * tfmp_base_address = 17 (17 equals to 0x11 in hexadecimal)
    * tfmp_bus = 0
    * tfmp_num_devices = 3 (note that the devices must have consecutive addresses)

* Launch the node:
    ```bash
    roslaunch tf_mini_plus_ros tf_mini_plus.launch
    ```
    The node now publishes all sensor range data over the topic /range_sensor/sensor***X*** which is a msg type sensor_msgs/Range

## Accessing I2C device for non root users
When trying to access the i2c bus from any of the nodes of the package from a non root user. It is common that the system throws an error that looks like the following:
```bash
[ WARN] [1580230037.358886488]: ERROR opening sensor on bus 1 and address 0x13 -
>[i2c]Could not open the file: Permission denied
```
In this case, it will be necesary to prun the following commands:
