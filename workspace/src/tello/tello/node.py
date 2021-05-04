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
from . import tello
from tello_msg.msg import FlightStatus
from std_msgs.msg import Empty, UInt8, UInt8, Bool
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# Tello ROS node class, inherits from the Tello controller object.
#
# Can be configured to be used by multiple drones, publishes, all data collected from the drone and provides control using ROS messages.
class TelloNode(tello.Tello):
    def __init__(self, node):
        self.node = node

        self.tello = Tello()

        # Connection parameters
        self.node.declare_parameter('connect_timeout', 10.0)
        self.node.declare_parameter('tello_ip', '192.168.10.1')
        self.node.declare_parameter('tf_base', 'map')
        self.node.declare_parameter('tf_drone', 'drone')

        # Connection parameters
        self.connect_timeout = float(self.node.get_parameter('connect_timeout').value)
        self.tello_ip = str(self.node.get_parameter('tello_ip').value)

        # TF parameters
        self.tf_base = str(self.node.get_parameter('tf_base').value)
        self.tf_drone = str(self.node.get_parameter('tf_drone').value)

        # OpenCV bridge
        self.bridge = CvBridge()

        super().__init__(self.tello_ip)

        # Connect to drone
        self.node.get_logger().info('Tello: Connecting to drone')
        self.connect()

        try:
            self.wait_for_connection(timeout=self.connect_timeout)
        except Exception as err:
            self.terminate(err)
            return

        self.node.get_logger().info('Tello: Connected to drone')

        # Setup ROS publishers
        self.pub_image_raw = self.node.create_publisher(Image, 'image_raw', 10)
        self.pub_camera_info = self.node.create_publisher(CameraInfo, 'camera_info', 10)
        self.pub_status = self.node.create_publisher(FlightStatus, 'status', 10)
        self.pub_imu = self.node.create_publisher(Imu, 'imu', 10)
        self.pub_battery = self.node.create_publisher(BatteryState, 'battery', 10)
        self.pub_temperature = self.node.create_publisher(Temperature, 'temperature', 10)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', 10)

        # Setup TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)

        # Setup ROS subscribers
        self.sub_emergency = self.node.create_subscription(Empty, 'emergency', self.cb_stop, 10)
        self.sub_takeoff = self.node.create_subscription(Empty, 'takeoff', self.cb_takeoff, 10)
        self.sub_land = self.node.create_subscription(Empty, 'land', self.cb_land, 10)
        self.sub_cmd_vel = self.node.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, 10)

        # Subscribe data from drone
        # self.subscribe(self.EVENT_FLIGHT_DATA, self.cb_drone_flight_data)
        # self.subscribe(self.EVENT_LOG_DATA, self.cb_drone_odom_log_data)
        # self.subscribe(self.EVENT_LIGHT, self.cb_drone_light_data)

        self.node.get_logger().info('Tello: Driver node ready')
    

    """
    Start video capture thread.
    """
    def start_video_capture(self):
        self.tello.streamon()
        frame_read = self.tello.get_frame_read()

        def video_capture_thread():
            # Create a video write
            height, width, _ = frame_read.frame.shape

            while True:
                frame_read.frame
                time.sleep(1 / 30)

            video.release()

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        recorder = Thread(target=video_capture_thread)
        recorder.start()

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.state = self.STATE_QUIT
        self.node.get_logger().error(str(err))
        rclpy.shutdown()
        self.quit()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.emergency()

    # Drone takeoff message control
    def cb_takeoff(self, msg):
        self.tello.takeoff()

    # Land the drone message callback
    def cb_land(self, msg):
        self.tello.land()

    # Callback method called when the drone sends light data
    def cb_drone_light_data(self, event, sender, data, **args):
        # pp = pprint.PrettyPrinter(width=41, compact=True)
        # pp.pprint("------------LDATA--------------"
        # pp.pprint(data.__dict__)
        # pp.pprint("----------- MVO DATA-----------"
        # pp.pprint(data.mvo.__dict__)
        # pp.pprint("----------- IMU DATA-----------"
        # pp.pprint(data.imu.__dict__)
        # pp.pprint("-------------------------------"

        return

    # Callback method called when the drone sends odom info
    def cb_drone_odom_log_data(self, event, sender, data, **args):
        # pp = pprint.PrettyPrinter(width=41, compact=True)
        # pp.pprint("----------- MVO DATA-----------")
        # pp.pprint(data.mvo.__dict__)
        # pp.pprint(data.mvo.log.__dict__)
        # pp.pprint("----------- IMU DATA-----------")
        # pp.pprint(data.imu.__dict__)
        # pp.pprint(data.imu.log.__dict__)
        # pp.pprint("----------LOG DATA-------------")
        # pp.pprint(data.log.__dict__)
        # pp.pprint("-------------------------------")

        # # Position data
        # position = (data.mvo.pos_x, data.mvo.pos_y, data.mvo.pos_z)
        # quaternion = (data.imu.q0, data.imu.q1, data.imu.q2, data.imu.q3)
        # euler = (data.imu.gyro_x, data.imu.gyro_y, data.imu.gyro_z)

        # # Publish drone transform
        # t = TransformStamped()
        # t.header.stamp = self.node.get_clock().now().to_msg()
        # t.header.frame_id = self.tf_base
        # t.child_frame_id = self.tf_drone
        # t.transform.translation.x = data.mvo.pos_x
        # t.transform.translation.y = data.mvo.pos_y
        # t.transform.translation.z = data.mvo.pos_z
        # self.tf_broadcaster.sendTransform(t)

        # # Publish odom data
        # odom_msg = Odometry()
        # odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        # odom_msg.header.frame_id = self.tf_base
        # odom_msg.pose.pose.position.x = data.mvo.pos_x
        # odom_msg.pose.pose.position.y = data.mvo.pos_y
        # odom_msg.pose.pose.position.z = data.mvo.pos_z
        # odom_msg.twist.twist.linear.x = data.mvo.vel_x
        # odom_msg.twist.twist.linear.y = data.mvo.vel_y
        # odom_msg.twist.twist.linear.z = data.mvo.vel_z
        # self.pub_odom.publish(odom_msg)

        # # Publish IMU data
        # imu_msg = Imu()
        # imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        # imu_msg.header.frame_id = self.tf_drone
        # imu_msg.linear_acceleration.x = data.imu.acc_x
        # imu_msg.linear_acceleration.y = data.imu.acc_y
        # imu_msg.linear_acceleration.z = data.imu.acc_z
        # imu_msg.angular_velocity.x = data.imu.gyro_x
        # imu_msg.angular_velocity.y = data.imu.gyro_y
        # imu_msg.angular_velocity.z = data.imu.gyro_z
        # imu_msg.orientation.x = data.imu.q0
        # imu_msg.orientation.y = data.imu.q1
        # imu_msg.orientation.z = data.imu.q2
        # imu_msg.orientation.w = data.imu.q3
        # self.pub_imu.publish(imu_msg)

    # Callback called every time the drone sends information
    def cb_drone_flight_data(self, event, sender, data, **args):
        # # Publish battery message
        # battery_msg = BatteryState()
        # battery_msg.percentage = float(data.battery_percentage)
        # battery_msg.voltage = 3.8
        # battery_msg.design_capacity = 1.1
        # battery_msg.present = True
        # battery_msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
        # battery_msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
        # self.pub_battery.publish(battery_msg)

        # # Publish temperature data
        # temperature_msg = Temperature()
        # temperature_msg.header.frame_id = self.tf_drone
        # temperature_msg.temperature = float(data.temperature_height)
        # self.pub_temperature.publish(temperature_msg)

        # # Publish flight data
        # msg = FlightStatus()
        # msg.imu_state = data.imu_state
        # msg.pressure_state = data.pressure_state
        # msg.down_visual_state = data.down_visual_state
        # msg.gravity_state = data.gravity_state
        # msg.wind_state = data.wind_state
        # msg.drone_hover = data.drone_hover
        # msg.factory_mode = data.factory_mode
        # msg.imu_calibration_state = data.imu_calibration_state
        # msg.fly_mode = data.fly_mode #1 Landed | 6 Flying
        # msg.camera_state = data.camera_state
        # msg.electrical_machinery_state = data.electrical_machinery_state

        # msg.front_in = data.front_in
        # msg.front_out = data.front_out
        # msg.front_lsc = data.front_lsc

        # msg.power_state = data.power_state
        # msg.battery_state = data.battery_state
        # msg.battery_low = data.battery_low
        # msg.battery_lower = data.battery_lower
        # msg.battery_percentage = data.battery_percentage
        # msg.drone_battery_left = data.drone_battery_left

        # msg.light_strength = data.light_strength

        # msg.fly_speed = float(data.fly_speed)
        # msg.east_speed = float(data.east_speed)
        # msg.north_speed = float(data.north_speed)
        # msg.ground_speed = float(data.ground_speed)
        # msg.fly_time = float(data.fly_time)
        # msg.drone_fly_time_left = float(data.drone_fly_time_left)

        # msg.wifi_strength = data.wifi_strength
        # msg.wifi_disturb = data.wifi_disturb

        # msg.height = float(data.height)
        # msg.temperature_height = float(data.temperature_height)

        # self.pub_status.publish(msg)

    # Callback for cmd_vel messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    def cb_cmd_vel(self, msg):
        self.tello.send_rc_control(msg.linear.x, msg.linear.y, msg.linear.z, msg.agular.z)


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
