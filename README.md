# robot_minor
**Welcome to the GitHub for the Challenge 2 + 3 Robot LiDAR.\n
Here you will find the information on how to install, setup and use the software for this project.\n
Good luck and read well!**

## Buildup
- Setting up Git Repository
- Dependencies

## Prequisits
You need to have ROS2 Jazzy installed with Colcon and Python3
Tutorial ROS2 Jazzy install: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
Tutorial Colcon install: 
Here is a quick turturial how to intall colcon, python and 
```
sudo apt install python3-colcon-common-extensions
```
Installing python3 and other python serveses:
```
sudo apt update
sudo apt install -y python3-pip python3-venv
```
Check if python is downloaded correctly
```
python3 --version
```
An result like this should show up:
```
Python 3.12.3
```
## Setting up Git Repository
First step is to make the workspace to pull all the code in.

## Bluetooth setup for controller
For using the Mecanum_joystick package, the option is to use a Sony PS4 controller instead of the keyboard with the teleop_twist_keyboard node. The controller needs to be connected to the Raspberry Pi5 itself. This is harder to do than normal due to the absence of an UI. You need to find the MAC-address of the controller. This is best to find by connecting to the controller first on your host Ubuntu machine. You can find the MAC-address then in the properties in the blueotth section of your settings.

**Pairing mode enable for Sony PS4 controller: hold [share button] + [PS] button until the blue LED starts flashing rappidly**

Install the bluetooth dependencies
```
sudo apt update
sudo apt install bluetooth bluez bluez-tools
```
Enable and start the bluetooth services
```
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```
Start the bleutooth services
```
bluetoothctl
```
**Now is the moment to get the controller into pairing mode**\n
Inside the new prompt enter:
```
agent on
default-agent
scan on
```
Now a list of all bluetooth devices will show op. You will find something like this: 
```
[NEW] Device XX:XX:XX:XX:XX:XX Wireless Controller
```
Copy over the MAC-address of the controller that is seen in the list, or that you have aquired before into the following commands:
```
pair XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
```
