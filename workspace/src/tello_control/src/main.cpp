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

using namespace std::chrono_literals;

class TelloControl : public rclcpp::Node
{
	public:
		TelloControl() : Node("control"), count(0)
		{
			publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
			publisher_land = this->create_publisher<std_msgs::msg::Empty>("land", 10);
			publisher_takeoff = this->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
			publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			

			timer = this->create_wall_timer(1ms, std::bind(&TelloControl::timer_callback, this));
		}

	private:
		/**
		 * Store the last key pressed when controlling the drone manually.
		 *
		 * Used to detect changes in key pressed.
		 */
		int last_key = NO_KEY;

		/**
		 * Speed of the drone in manual control mode.
		 */
		double manual_speed = 0.6;

		rclcpp::TimerBase::SharedPtr timer;

		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;

		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_takeoff;

		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_land;

		size_t count;

		/**
		 * Method to control the drone using the keyboard inputs.
		 *
		 * @param key Keycode received.
		 */
		void manualControl(int key)
		{
			// Only send data on changes
			if(key != last_key)
			{
				geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

				if(key == KEY_UP) {msg.linear.x = manual_speed;}
				if(key == KEY_DOWN) {msg.linear.x = -manual_speed;}
				if(key == KEY_LEFT) {msg.linear.y = manual_speed;}
				if(key == KEY_RIGHT) {msg.linear.y = -manual_speed;}
				if(key == KEY_W) {msg.linear.z = manual_speed;}
				if(key == KEY_S) {msg.linear.z = -manual_speed;}
				if(key == KEY_A) {msg.angular.z = manual_speed;}
				if(key == KEY_D) {msg.angular.z = -manual_speed;}

				publisher_velocity->publish(msg);
			}

			// Store last key for diffs
			last_key = key;
		}
	
		void timer_callback()
		{

			cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
			cv::namedWindow("Tello", cv::WINDOW_AUTOSIZE);
			cv::imshow("Tello", image);	

			int key = cv::waitKey(1);

			// Takeoff
			if(key == (int)('t'))
			{
				std_msgs::msg::Empty empty = std_msgs::msg::Empty();
				publisher_takeoff->publish(empty);

				std::cout << "Takeoff Message" << std::endl;
			}
			// Land
			else if(key == (int)('l'))
			{
				std_msgs::msg::Empty empty = std_msgs::msg::Empty();
				publisher_land->publish(empty);
			}
			else
			{
				manualControl(key);
			}

			// auto message = std_msgs::msg::String();
			// message.data = "Hello, world! " + std::to_string(count++);
			// publisher->publish(message);
		}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TelloControl>());
	rclcpp::shutdown();
	return 0;
}