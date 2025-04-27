GPS_DRIVER:

Create gps_driver package - Consists of python files

Update package.xml

colcon build --packages-select gps_driver
source install/setup.bash


ros2 pkg list | grep gps_driver


ros2 run gps_driver gps_driver --port /dev/ttyUSB0


ros2 interface list | grep gps_msgs
ros2 interface show gps_msgs/msg/GpsMsg



VIRTUAL PORT:

Check virtual port:

ls -l /dev/pts/*




LAUNCH:
ros2 launch gps_driver gps_launch.py port:=/dev/pts/1







GPS_MSG:
Create gps_msg package-Consists of msg files

Create msg folder/ GpsMsg.msg

Create CmakeLists.txt

Update package.xml

colcon build --packages-select gps_msgs
source install/setup.bash

ros2 interface list | grep gps_msgs





SENSOR_EMULATOR:

python3 serial_emulator.py --file GPS_Chicago.txt

