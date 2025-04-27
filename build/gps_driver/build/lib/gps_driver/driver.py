#!/usr/bin/env python3
# -- coding: utf-8 --

import rclpy
from rclpy.node import Node
import serial
import utm
import glob
import sys
from gps_msgs.msg import GpsMsg
from std_msgs.msg import Header

class GPSPublisher(Node):
    """
    ROS2 node that reads GPS data from a serial connection,
    processes it, and publishes it as a ROS message.
    """

    def __init__(self, serial_port):
        super().__init__('gps_publisher')

        # Declare ROS parameters for customization
        self.declare_parameter('baud_rate', 4800)
        self.declare_parameter('enable_debug_logs', False)

        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug_logs = self.get_parameter('enable_debug_logs').value

        self.publisher = self.create_publisher(GpsMsg, '/gps', 10)
        self.last_gps_time = self.get_clock().now()

        # Attempt to connect to the GPS device via serial
        try:
            self.gps_serial = serial.Serial(serial_port, baudrate=self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to GPS device at {serial_port} with baud rate {self.baud_rate}")
        except serial.SerialException as err:
            self.get_logger().fatal(f"Failed to open serial port: {err}")
            self.gps_serial = None
            return

        # Create a timer to continuously read GPS data
        self.timer = self.create_timer(0.1, self.read_gps_data)

    def read_gps_data(self):
        """
        Reads and processes GPS data, converts coordinates, and publishes messages.
        """
        if not self.gps_serial:
            return

        current_time = self.get_clock().now()
        time_since_last_gps = (current_time - self.last_gps_time).nanoseconds / 1e9

        if time_since_last_gps > 5:
            self.get_logger().warn("No GPS data received for more than 5 seconds!")

        try:
            if self.debug_logs:
                self.get_logger().debug("Attempting to read GPS data...")

            raw_data = self.gps_serial.readline().decode('utf-8').strip()
            if raw_data.startswith("$GPGGA"):
                self.last_gps_time = current_time
                self.get_logger().info(f"Received GPS Data: {raw_data}")

                gps_info = self.extract_gps_info(raw_data)
                if gps_info:
                    utm_values = self.convert_to_utm(*gps_info[1:5])
                    if utm_values:
                        message = self.construct_gps_msg(gps_info, utm_values)
                        self.publisher.publish(message)
        except Exception as err:
            self.get_logger().error(f"Error while processing GPS data: {err}")

    def extract_gps_info(self, gpgga_data):
        """
        Extracts and parses GPS information from a GPGGA sentence.
        """
        data_parts = gpgga_data.split(',')
        if len(data_parts) < 15:
            self.get_logger().error("Incomplete GPGGA string received.")
            return None

        try:
            utc_timestamp = float(data_parts[1]) if data_parts[1] else 0.0
            raw_latitude = data_parts[2] or '0'
            lat_direction = data_parts[3]
            raw_longitude = data_parts[4] or '0'
            lon_direction = data_parts[5]
            altitude = float(data_parts[9]) if data_parts[9] else 0.0
            hdop = float(data_parts[8]) if data_parts[8] else 0.0

            return utc_timestamp, raw_latitude, lat_direction, raw_longitude, lon_direction, altitude, hdop
        except Exception as err:
            self.get_logger().error(f"Failed to parse GPGGA data: {err}")
            return None

    def convert_to_utm(self, raw_lat, lat_dir, raw_lon, lon_dir):
        """
        Converts latitude and longitude values to UTM coordinates.
        """
        try:
            lat_deg = float(raw_lat[:2]) + float(raw_lat[2:]) / 60
            if lat_dir == 'S':
                lat_deg = -lat_deg

            lon_deg = float(raw_lon[:3]) + float(raw_lon[3:]) / 60
            if lon_dir == 'W':
                lon_deg = -lon_deg

            utm_values = utm.from_latlon(lat_deg, lon_deg)
            return lat_deg, lon_deg, utm_values[0], utm_values[1], utm_values[2], utm_values[3]
        except Exception as err:
            self.get_logger().error(f"Failed to convert lat/lon to UTM: {err}")
            return None

    def construct_gps_msg(self, gps_info, utm_info):
        """
        Creates a ROS message containing GPS and UTM data.
        """
        gps_msg = GpsMsg()
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "GPS1_FRAME"

        gps_msg.latitude, gps_msg.longitude = utm_info[:2]
        gps_msg.utm_easting, gps_msg.utm_northing = utm_info[2:4]
        gps_msg.altitude = gps_info[5]
        gps_msg.hdop = gps_info[6]
        gps_msg.zone = str(utm_info[4])
        gps_msg.letter = utm_info[5]

        return gps_msg

def main(args=None):
    """
    Initializes the ROS2 node and manages its execution.
    """
    rclpy.init(args=args)

    # Detect available serial port
    available_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/pts/*')
    port = available_ports[0] if available_ports else None

    if not port:
        print("Error: No available serial port detected!")
        return

    gps_publisher = GPSPublisher(port)

    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        print("\nShutting down GPS publisher...")
    finally:
        if gps_publisher.gps_serial and gps_publisher.gps_serial.is_open:
            gps_publisher.gps_serial.close()
            print("Closed GPS serial connection.")
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

