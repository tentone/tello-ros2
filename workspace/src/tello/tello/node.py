#!/usr/bin/env python3

import pprint
import math
import rclpy
import threading
import numpy
import time
import av
import tf2_ros
import cv2
import time

from djitellopy import Tello

from rclpy.node import Node
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# Tello ROS node class, inherits from the Tello controller object.
#
# Can be configured to be used by multiple drones, publishes, all data collected from the drone and provides control using ROS messages.
class TelloNode():
    def __init__(self, node):
        # ROS node
        self.node = node

        # Declare parameters
        self.node.declare_parameter('connect_timeout', 10.0)
        self.node.declare_parameter('tello_ip', '192.168.10.1')
        self.node.declare_parameter('tf_base', 'map')
        self.node.declare_parameter('tf_drone', 'drone')

        # Get parameters
        self.connect_timeout = float(self.node.get_parameter('connect_timeout').value)
        self.tello_ip = str(self.node.get_parameter('tello_ip').value)
        self.tf_base = str(self.node.get_parameter('tf_base').value)
        self.tf_drone = str(self.node.get_parameter('tf_drone').value)

        # Configure drone connection
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        # Connect to drone
        self.node.get_logger().info('Tello: Connecting to drone')

        self.tello = Tello()
        self.tello.connect()

        self.node.get_logger().info('Tello: Connected to drone')

        # Position estimative based on IMU
        self.position = [0.0, 0.0, 0.0]

        # Publishers and subscribers
        self.setup_publishers()
        self.setup_subscribers()

        # Processing threads
        self.start_video_capture()
        self.start_tello_status()
        self.start_tello_odom()

        self.node.get_logger().info('Tello: Driver node ready')

    # Setup ROS publishers of the node.
    def setup_publishers(self):
        self.pub_image_raw = self.node.create_publisher(Image, 'image_raw', 1)
        self.pub_camera_info = self.node.create_publisher(CameraInfo, 'camera_info', 1)
        self.pub_status = self.node.create_publisher(TelloStatus, 'status', 1)
        self.pub_id = self.node.create_publisher(TelloID, 'id', 1)
        self.pub_imu = self.node.create_publisher(Imu, 'imu', 1)
        self.pub_battery = self.node.create_publisher(BatteryState, 'battery', 1)
        self.pub_temperature = self.node.create_publisher(Temperature, 'temperature', 1)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', 1)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
    
    # Setup the topic subscribers of the node.
    def setup_subscribers(self):
        self.sub_emergency = self.node.create_subscription(Empty, 'emergency', self.cb_emergency, 1)
        self.sub_takeoff = self.node.create_subscription(Empty, 'takeoff', self.cb_takeoff, 1)
        self.sub_land = self.node.create_subscription(Empty, 'land', self.cb_land, 1)
        self.sub_control = self.node.create_subscription(Twist, 'control', self.cb_control, 1)
        self.sub_flip = self.node.create_subscription(String, 'flip', self.cb_flip, 1)
        self.sub_wifi_config = self.node.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 1)

    # Start drone info thread
    def start_tello_odom(self, rate=0.1):
        def status_odom():
            while True:
                # Position
                self.position[0] += self.tello.get_speed_x()
                self.position[1] += self.tello.get_speed_y()
                self.position[2] += self.tello.get_speed_z()

                # TF
                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = self.tf_base
                t.child_frame_id = self.tf_drone
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = (self.tello.get_barometer()) / 100.0
                self.tf_broadcaster.sendTransform(t)
                
                # Odometry
                if self.pub_odom.get_subscription_count() > 0:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.node.get_clock().now().to_msg()
                    odom_msg.header.frame_id = self.tf_base
                    odom_msg.pose.pose.position.x = 0.0
                    odom_msg.pose.pose.position.y = 0.0
                    odom_msg.pose.pose.position.z = 0.0
                    odom_msg.twist.twist.linear.x = self.tello.get_speed_x()
                    odom_msg.twist.twist.linear.y = self.tello.get_speed_y()
                    odom_msg.twist.twist.linear.z = self.tello.get_speed_z()
                    self.pub_odom.publish(odom_msg)
                
                time.sleep(rate)

        thread = threading.Thread(target=status_odom)
        thread.start()
        return thread

    # Start drone info thread
    def start_tello_status(self, rate=0.5):
        def status_loop():
            while True:
                # Battery
                if self.pub_battery.get_subscription_count() > 0:
                    battery_msg = BatteryState()
                    battery_msg.header.frame_id = self.tf_drone
                    battery_msg.percentage = float(self.tello.get_battery())
                    battery_msg.voltage = 3.8
                    battery_msg.design_capacity = 1.1
                    battery_msg.present = True
                    battery_msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
                    battery_msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
                    self.pub_battery.publish(battery_msg)

                # Temperature
                if self.pub_temperature.get_subscription_count() > 0:
                    temperature_msg = Temperature()
                    temperature_msg.header.frame_id = self.tf_drone
                    temperature_msg.temperature = self.tello.get_temperature()
                    self.pub_temperature.publish(temperature_msg)

                # IMU
                if self.pub_imu.get_subscription_count() > 0:
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.node.get_clock().now().to_msg()
                    imu_msg.header.frame_id = self.tf_drone
                    imu_msg.linear_acceleration.x = self.tello.get_acceleration_x()
                    imu_msg.linear_acceleration.y = self.tello.get_acceleration_y()
                    imu_msg.linear_acceleration.z = self.tello.get_acceleration_z()
                    self.pub_imu.publish(imu_msg)

                # Tello Status
                if self.pub_status.get_subscription_count() > 0:
                    msg = TelloStatus()
                    msg.acceleration.x = self.tello.get_acceleration_x()
                    msg.acceleration.y = self.tello.get_acceleration_y()
                    msg.acceleration.z = self.tello.get_acceleration_z()

                    msg.speed.x = self.tello.get_speed_x()
                    msg.speed.y = self.tello.get_speed_y()
                    msg.speed.z = self.tello.get_speed_z()

                    msg.barometer = self.tello.get_barometer()
                    msg.distance_tof = self.tello.get_distance_tof()

                    msg.fligth_time = self.tello.get_flight_time()

                    msg.battery = self.tello.get_battery()

                    msg.highest_temperature = self.tello.get_highest_temperature()
                    msg.lowest_temperature = self.tello.get_lowest_temperature()
                    msg.temperature = self.tello.get_temperature()

                    msg.pitch = self.tello.get_pitch()
                    msg.roll = self.tello.get_roll()
                    msg.yaw = self.tello.get_yaw()

                    msg.wifi_snr = self.tello.query_wifi_signal_noise_ratio()

                    self.pub_status.publish(msg)

                # Tello ID
                if self.pub_id.get_subscription_count() > 0:
                    msg = TelloID()
                    msg.sdk_version = self.tello.query_sdk_version()
                    msg.serial_number = self.tello.query_serial_number()
                    self.pub_id.publish(msg)
                
                # Sleep
                time.sleep(rate)

        thread = threading.Thread(target=status_loop)
        thread.start()
        return thread


    # Start video capture thread.
    def start_video_capture(self, rate=1.0/30.0):
        # Enable tello stream
        self.tello.streamon()

        # OpenCV bridge
        self.bridge = CvBridge()

        def video_capture_thread():
            frame_read = self.tello.get_frame_read()

            while True:
                # Get frame from drone
                frame = frame_read.frame

                # Publish opencv frame using CV bridge
                msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'bgr8')
                msg.header.frame_id = self.tf_drone
                self.pub_image_raw.publish(msg)

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.node.get_logger().error(str(err))
        self.tello.end()
        rclpy.shutdown()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.emergency()

    # Drone takeoff message control
    def cb_takeoff(self, msg):
        self.tello.takeoff()

    # Land the drone message callback
    def cb_land(self, msg):
        self.tello.land()

    # Control messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    #
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def cb_control(self, msg):
        self.tello.send_rc_control(int(msg.linear.x), int(msg.linear.y), int(msg.linear.z), int(msg.angular.z))

    # Configure the wifi credential that should be used by the drone.
    #
    # The drone will be restarted after the credentials are changed.
    def cb_wifi_config(self, msg):
        self.tello.set_wifi_credentials(msg.ssid, msg.password)
    
    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        self.tello.flip(msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tello')
    drone = TelloNode(node)

    rclpy.spin(node)

    drone.cb_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
