#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

#define KEY_UP 82
#define KEY_DOWN 84
#define KEY_LEFT 81
#define KEY_RIGHT 83

#define KEY_ENTER 13
#define KEY_SPACE 32

using namespace std::chrono_literals;

class TelloControl : public rclcpp::Node
{
	public:
		/**
		 * Store the last key pressed when controlling the drone manually.
		 *
		 * Used to detect changes in key pressed.
		 */
		int last_key = NO_KEY;

		/**
		 * Timer used to control the execution speed of the node.
		 */
		rclcpp::TimerBase::SharedPtr timer;

		/**
		 * Publish drone control velocity.
		 */
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;

		/**
		 * Publish takeoff control.
		 */
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_takeoff;

		/**
		 * Publisher for drone flip commands.
		 */
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_flip;

		/**
		 * Publisher for landing controls.
		 */
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_land;

		/**
		 * Publisher for emergency stop.
		 */
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_emergency;

		/**
		 * Construct a new Tello Control object
		 */
		TelloControl() : Node("control")
		{
			publisher_land = this->create_publisher<std_msgs::msg::Empty>("land", 1);
			publisher_flip = this->create_publisher<std_msgs::msg::String>("flip", 1);
			publisher_takeoff = this->create_publisher<std_msgs::msg::Empty>("takeoff", 1);
			publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("control", 1);
			publisher_emergency = this->create_publisher<std_msgs::msg::Empty>("emergency", 1);

			timer = this->create_wall_timer(1ms, std::bind(&TelloControl::timerCallback, this));
		}

		/**
		 * Method to control the drone using the keyboard inputs.
		 *
		 * @param key Keycode received.
		 */
		void manualControl(int key)
		{
			// Speed of the drone in manual control mode.
			double manual_speed = 50;

			geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
		
			if(key == KEY_LEFT) {msg.linear.x = -manual_speed;}
			if(key == KEY_RIGHT) {msg.linear.x = +manual_speed;}
			if(key == KEY_UP) {msg.linear.y = manual_speed;}
			if(key == KEY_DOWN) {msg.linear.y = -manual_speed;}
			if(key == (int)('w')) {msg.linear.z = manual_speed;}
			if(key == (int)('s')) {msg.linear.z = -manual_speed;}
			if(key == (int)('a')) {msg.angular.z = -manual_speed;}
			if(key == (int)('d')) {msg.angular.z = manual_speed;}

			publisher_velocity->publish(msg);
		}

		void timerCallback()
		{
			cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
			cv::namedWindow("Tello", cv::WINDOW_AUTOSIZE);
			cv::imshow("Tello", image);	

			int key = cv::waitKey(15);
			
			if (key != NO_KEY)
			{
				// Takeoff
				if(key == (int)('t'))
				{
					publisher_takeoff->publish(std_msgs::msg::Empty());
				}
				// Land
				else if(key == (int)('l'))
				{
					publisher_land->publish(std_msgs::msg::Empty());
				}
				// Flip
				else if(key == (int)('f'))
				{
					std_msgs::msg::String msg = std_msgs::msg::String();
					msg.data = 'f';
					publisher_flip->publish(msg);
				}
				// Emergency Stop
				else if(key == (int)('e'))
				{
					publisher_emergency->publish(std_msgs::msg::Empty());
				}
				else
				{
					manualControl(key);
				}
			}

			// Store last key for diffs
			last_key = key;
		}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TelloControl>());
	rclcpp::shutdown();
	return 0;
}