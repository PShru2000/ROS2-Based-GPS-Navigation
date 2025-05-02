# ROS2-Based GPS Navigation

## Goal

This project aims to record, publish, and analyze GPS data using a USB-based GNSS puck in a ROS2 environment. It converts raw NMEA GPGGA data to UTM coordinates and publishes them via a custom ROS2 message. The data is also stored in ROS2 bag files for further analysis.

## Features

### GPS Driver (`driver.py`)
- Reads GPGGA sentences from a USB serial port.
- Parses latitude, longitude, altitude, HDOP, UTC time, UTM zone and letter.
- Converts latitude and longitude to UTM using the `utm` package.
- Publishes data using a custom ROS2 message on the `/gps` topic.

### Custom ROS2 Message (`GpsMsg.msg`)

```text
std_msgs/Header header
float64 latitude
float64 longitude
float64 altitude
float64 hdop
float64 utm_easting
float64 utm_northing
string utc
string zone
string letter
````

### Launch File (`gps_launch.py`)

* Accepts a serial port as a launch argument
* Launches the GPS driver node with the specified port

## Installation

```bash
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions python3-pip
pip3 install utm pyserial
```

## Building the Project

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YourUsername/ROS2-Based-GPS-Navigation.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running the Driver Code

```bash
ros2 launch gps_driver gps_launch.py port:=/dev/ttyUSB0
```

**Note**
Replace `/dev/ttyUSB0` with the actual serial port to which your GPS puck is connected.


## Recording Data

Use the ROS2 bag tool to record GPS data:

```bash
ros2 bag record /gps
````

## Data Analysis

Use Python for offline data analysis:

```bash
pip install bagpy pandas matplotlib
```

### Analysis

* **Scatter plot**: Northing vs Easting
* **Histogram**: Error from known position
* **Line fitting**: Deviation from expected path
* **Plot**: Altitude vs Time

## Sources of GPS Error

* **Wind**: Minor variation in GPS readings
* **Rain**: Weakens satellite signal, increases drift
* **Buildings/Trees**: Cause multipath interference

## Conclusion

This ROS2-based project demonstrates real-time GPS data acquisition, processing, and publication using a USB GNSS puck. It supports flexible hardware configuration, structured data publishing, and statistical analysis, laying the groundwork for integration into broader autonomous navigation systems.
