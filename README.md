# robot_minor
**Welcome to the GitHub for the Challenge 2 + 3 Robot LiDAR.\n
Here you will find the information on how to install, setup and use the software for this project.\n
Good luck and read well!**

## Buildup
- Setting up Git Repository
- Dependencies

## Prequisits
You need to have ROS2 Jazzy installed with Colcon and Python3
Tutorial ROS2 Jazzy install: 
Tutorial Colcon install: 
## Setting up Git Repository
First step is to make the workspace to pull all the code in.

## Bluetooth setup for controller
For using the Mecanum_joystick package, the option is to use a Sony PS4 controller instead of the keyboard with the teleop_twist_keyboard node. The controller needs to be connected to the Raspberry Pi5 itself. This is harder to do than normal due to the absence of an UI. You need to find the MAC-address of the controller. This is best to find by connecting to the controller first on your host Ubuntu machine. You can find the MAC-address then in the properties in the blueotth section of your settings.


```
bluetoothctl
scan on
```

```
agent on
default-agent
scan on
```
```
[NEW] Device XX:XX:XX:XX:XX:XX Wireless Controller
```
