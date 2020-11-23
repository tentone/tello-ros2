#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define DEBUG true

#define PI 3.14159265359
#define PI2 PI * 2.0
#define DEG_TO_RAD PI / 180

#define NO_KEY -1

#define KEY_NUM_0 48
#define KEY_NUM_1 49
#define KEY_NUM_2 50
#define KEY_NUM_3 51
#define KEY_NUM_4 52
#define KEY_NUM_5 53
#define KEY_NUM_6 54
#define KEY_NUM_7 55
#define KEY_NUM_8 56
#define KEY_NUM_9 57

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_LEFT 68
#define KEY_RIGHT 67

#define KEY_ENTER 10
#define KEY_SPACE 32

#define KEY_Q 113
#define KEY_W 119
#define KEY_A 97
#define KEY_S 115
#define KEY_D 100
#define KEY_E 101
#define KEY_T 116
#define KEY_L 108
#define KEY_H 104

#define WAYPOINT_WAITING 0
#define WAYPOINT_HAS_WAYPOINT 1
#define WAYPOINT_NAVIGATING 2
#define WAYPOINT_FINISHED 3

/**
 * Manual control mode using the keyboard ARROWS, WASD, L, T SPACE keys.
 */
#define MODE_MANUAL 0

/**
 * Follow person detected using an external node, that returns the velocity that should be applied.
 */
#define MODE_FOLLOW_PERSON 1

/**
 * Move the drone torwards a way point received from and external node.
 */
#define MODE_WAYPOINT 2

using std::placeholders::_1;

/**
 * Get keyboard input key without blocking the application.
 *
 * @param timeout Timeout time in microseconds (1ms is 1000us)
 * @return Code of the key pressed.
 */
int getch(int timeout)
{
	struct termios oldattr, newattr;

	tcgetattr(STDIN_FILENO, &oldattr);
	newattr = oldattr;
	newattr.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newattr);

	while(1)
	{
		fd_set set;

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = timeout;

		FD_ZERO(&set);
		FD_SET(fileno(stdin), &set);

		int res = select(fileno(stdin) + 1, &set, NULL, NULL, &tv);
		if(res > 0)
		{
			break;
		}
		else
		{
			return NO_KEY;
		}
	}

	int ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);

	if(ch == 27 || ch == 79 || ch == 91)
	{
		return NO_KEY;
	}

	return ch;
}

class ControlNode : public rclcpp::Node
{
	public:
		/**
		 * Odometry provided by the drone, used to obtain the position and rotation of the drone.
		 */
		nav_msgs::msg::Odometry::SharedPtr odometry;

		/**
		 * IMU data provided by the drone.
		 */
		sensor_msgs::msg::Imu::SharedPtr imu;

		/**
		 * Waypoint target message. 
		 */
		geometry_msgs::msg::PointStamped::SharedPtr waypoint;

		/**
		 * Mode of the drone, defines its behavior.
		 */
		int mode = MODE_MANUAL;

		/**
		 * Store the last key pressed when controlling the drone manually.
		 *
		 * Used to detect changes in key pressed.
		 */
		int manual_last_key = NO_KEY;

		/**
		 * Speed of the drone in manual control mode.
		 */
		double manual_speed = 0.6;

		/**
		 * Waypoint navigation state
		 */
		int waypoint_state = WAYPOINT_WAITING;


		/**
		 * Indicates if the person is visible, if so the speed received is applied to the drone.
		 */
		bool person_visible = false;

		/**
		 * Indicates if the drone is currently moving in the direction of the person.
		 */
		bool person_drone_moving = false;

		rclcpp::TimerBase::SharedPtr timer;

		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff;
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_person_velocity;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_person_visible;
		rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_waypoint;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

		ControlNode(): Node("control")
		{
			timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControlNode::loop, this));

			// Subscribe topic parameters
			std::string param_sub_velocity, param_sub_visible;
			this->get_parameter_or<std::string>("sub_person_visible", param_sub_visible, "/person/visible");
			this->get_parameter_or<std::string>("sub_person_velocity", param_sub_velocity, "/person/velocity");

			std::string param_sub_waypoint;
			this->get_parameter_or<std::string>("sub_waypoint", param_sub_waypoint, "/clicked_point");

			std::string param_sub_odom, param_sub_imu;
			this->get_parameter_or<std::string>("sub_odom", param_sub_odom, "/tello/odom");
			this->get_parameter_or<std::string>("sub_imu", param_sub_imu, "/tello/imu");

			// Subscribers
			sub_person_velocity = this->create_subscription<geometry_msgs::msg::Twist>(param_sub_velocity, 1, std::bind(&ControlNode::personVelocitySubCallback, this, _1));
			sub_person_visible = this->create_subscription<std_msgs::msg::Bool>(param_sub_visible, 1, std::bind(&ControlNode::personVisibleSubCallback, this, _1));
			sub_waypoint = this->create_subscription<geometry_msgs::msg::PointStamped>(param_sub_waypoint, 1, std::bind(&ControlNode::waypointSubCallback, this, _1));
			sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(param_sub_odom, 1, std::bind(&ControlNode::odomSubCallback, this, _1));
			sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(param_sub_imu, 1, std::bind(&ControlNode::imuSubCallback, this, _1));
 

			// Publish topic parameters
			std::string param_pub_velocity, param_pub_takeoff, param_pub_land;
			this->get_parameter_or<std::string>("pub_takeoff", param_pub_takeoff, "/tello/takeoff");
			this->get_parameter_or<std::string>("pub_land", param_pub_land, "/tello/land");
			this->get_parameter_or<std::string>("pub_velocity", param_pub_velocity, "/tello/cmd_vel");

			// Publishers
			pub_takeoff = this->create_publisher<std_msgs::msg::Empty>(param_pub_takeoff, 10);
			pub_land = this->create_publisher<std_msgs::msg::Empty>(param_pub_land, 10);
			pub_velocity = this->create_publisher<geometry_msgs::msg::Twist>(param_pub_velocity, 10);
		}
	
	private:
		/**
		 * Set the mode of the drone control.
		 *
		 * @param m Mode to be set.
		 */
		void setMode(int mode)
		{
			this->mode = mode;

			#ifdef DEBUG
				std::cout << "Mode: " << this->mode << std::endl;
			#endif

			geometry_msgs::msg::Twist msg;
			this->pub_velocity->publish(msg);
		}

		/**
		 * Process callback contains the person velocity.
		 */
		void personVelocitySubCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
		{
			if(this->mode == MODE_FOLLOW_PERSON && person_visible)
			{
				auto vel_msg = geometry_msgs::msg::Twist();
				vel_msg.linear = msg->linear;
				vel_msg.angular = msg->angular;
				pub_velocity->publish(vel_msg);

				this->person_drone_moving = true;
			}
		}

		/**
		 * Process callback indicating if the person if visible.
		 */
		void personVisibleSubCallback(const std_msgs::msg::Bool::SharedPtr msg)
		{
			this->person_visible = msg->data;

			if(this->mode == MODE_FOLLOW_PERSON && !this->person_visible && this->person_drone_moving)
			{
				geometry_msgs::msg::Twist msg;
				this->pub_velocity->publish(msg);
				this->person_drone_moving = false;
			}
		}

		/**
		 * Callback method used to read and store the odometry value.
		 */
		void odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
		{
			this->odometry = msg;
		}

		/**
		 * Callback method used to read and store the odometry value.
		 */
		void imuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
		{
			this->imu = msg;

			#ifdef DEBUG
				tf2::Quaternion quaternion;
				tf2::fromMsg(imu->orientation, quaternion);

				double rot_x, rot_y, rot_z;
				tf2::Matrix3x3(quaternion).getRPY(rot_x, rot_y, rot_z);

				std::cout << "Rotation X: " << rot_z << std::endl;
				std::cout << "Rotation Y: " << rot_y << std::endl;
				std::cout << "Rotation Z: " << rot_z << std::endl;
			#endif
		}

		/**
		 * Received waypoint from external node and store it for navigation.
		 */
		void waypointSubCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
		{
			#ifdef DEBUG
				std::cout << "Received waypoint" << std::endl;
				std::cout << "X:" << msg->point.x << std::endl;
				std::cout << "Y:" << msg->point.y << std::endl;
				std::cout << "Z:" << msg->point.z << std::endl;
			#endif

			this->waypoint = msg;
			this->waypoint_state = WAYPOINT_HAS_WAYPOINT;
		}

		/**
		 * Dot product between two vectors.
		 */
		double dot(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b)
		{
			return a.x * b.x + a.y * b.y;
		}

		/**
		 * Cross product between two vectors.
		 */
		double cross(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b)
		{
			return a.x * b.y - a.y * b.x;
		}

		/**
		 * Move to position received from external node (e.g RViz)
		 */
		void waypointControl()
		{
			if(this->mode == MODE_WAYPOINT && (this->waypoint_state == WAYPOINT_HAS_WAYPOINT || this->waypoint_state == WAYPOINT_NAVIGATING))
			{
				geometry_msgs::msg::Twist msg;

				// Convert quaternion to euler rotation
				tf2::Quaternion quaternion;
				tf2::convert(imu->orientation, quaternion);

				double rot_x, rot_y, rot_z;
				tf2::Matrix3x3(quaternion).getRPY(rot_x, rot_y, rot_z);

				// Calculate angle between drone and waypoint
				double x = waypoint->point.x - odometry->pose.pose.position.x;
				double y = waypoint->point.y - odometry->pose.pose.position.y;
				double angle_points = atan2(y, x);

				// Ditance betwee points
				double distance = sqrt(pow(odometry->pose.pose.position.x - waypoint->point.x, 2) + pow(odometry->pose.pose.position.y - waypoint->point.y, 2));

				// Angle diff
				double angle_diff;

				if(angle_points < 0)
				{
					double a = abs(angle_points);
					double b = abs(rot_z);
					if(rot_z > 0) {angle_diff = a - b;}
					else {angle_diff = a + b;}
				}
				else if(angle_points > 0)
				{
					double b = abs(rot_z);
					if(rot_z > 0) {angle_diff = -angle_points - b;}
					else {angle_diff = -angle_points + b;}
				}

				if(angle_diff > PI) {angle_diff -= PI2;}
				if(angle_diff < -PI) {angle_diff += PI2;}

				if(distance > 0.2)
				{
					if(abs(angle_diff) < (20 * DEG_TO_RAD))
					{
						msg.linear.x = 0.5;
					}

					msg.angular.z = -(angle_diff / PI);
				}
				else
				{
					this->waypoint_state = WAYPOINT_FINISHED;
				}

				#ifdef DEBUG
					//std::cout << "Distance: " << distance << std::endl;
					std::cout << "Angle Points: " << angle_points << std::endl;
					std::cout << "Angle Drone: " << rot_z << std::endl;
					std::cout << "Angle Diff: " << angle_diff << std::endl;
				#endif

				this->pub_velocity->publish(msg);

				if(this->waypoint_state == WAYPOINT_HAS_WAYPOINT)
				{
					this->waypoint_state = WAYPOINT_NAVIGATING;
				}
			}
		}


		/**
		 * Method to control the drone using the keyboard inputs.
		 *
		 * @param key Keycode received.
		 */
		void manualControl(int key)
		{
			if(key == NO_KEY)
			{
				return;
			}

			// Only send data on changes
			if(key != this->manual_last_key)
			{
				geometry_msgs::msg::Twist msg;

				if(key == KEY_UP) {msg.linear.x = manual_speed;}
				if(key == KEY_DOWN) {msg.linear.x = -manual_speed;}
				if(key == KEY_LEFT) {msg.linear.y = manual_speed;}
				if(key == KEY_RIGHT) {msg.linear.y = -manual_speed;}
				if(key == KEY_W) {msg.linear.z = manual_speed;}
				if(key == KEY_S) {msg.linear.z = -manual_speed;}
				if(key == KEY_A) {msg.angular.z = manual_speed;}
				if(key == KEY_D) {msg.angular.z = -manual_speed;}

				this->pub_velocity->publish(msg);
			}

			// Store last key for diffs
			this->manual_last_key = key;
		}

		void loop()
		{
			int key = getch(10);

			if(key != NO_KEY)
			{
				std::cout << "Key pressed: " << key << std::endl;
			}

			// Toggle person follow mode
			if(key == KEY_NUM_1)
			{
				setMode(MODE_MANUAL);
			}
			else if(key == KEY_NUM_2)
			{
				setMode(MODE_FOLLOW_PERSON);
			}
			else if(key == KEY_NUM_3)
			{
				setMode(MODE_WAYPOINT);
			}

			// Takeoff
			if(key == KEY_T)
			{
				std_msgs::msg::Empty empty;
				pub_takeoff->publish(empty);
			}
			// Land
			else if(key == KEY_L)
			{
				std_msgs::msg::Empty empty;
				pub_land->publish(empty);
			}
			// Home
			else if(key == KEY_H)
			{
				geometry_msgs::msg::PointStamped msg;
				this->waypoint->point = msg.point;
				this->waypoint_state = WAYPOINT_HAS_WAYPOINT;
			}

			// Keyboard control
			if(this->mode == MODE_MANUAL)
			{
				manualControl(key);
			}
			else if(this->mode == MODE_WAYPOINT)
			{
				waypointControl();
			}
		}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControlNode>());
	rclcpp::shutdown();
	return 0;
}