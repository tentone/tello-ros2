#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>


#include<opencv2/core/core.hpp>

using std::placeholders::_1;

class MonocularSlamNode : public rclcpp::Node
{
	public:
		/**
		 * Timer used to control the execution speed of the node.
		 */
		rclcpp::TimerBase::SharedPtr timer;

		/**
		 * ORB slam instance.
		 */
		ORB_SLAM2::System* orb_slam;

		/**
		 * ROS bridge used to get the image from ROS message and convert it to CV image.
		 */
		cv_bridge::CvImagePtr image_ptr;

		/**
		 * Subscriber to receive the camera image to be processed.
		 */
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_image;

		MonocularSlamNode(ORB_SLAM2::System* pSLAM): Node("orbslam"), orb_slam(pSLAM)
		{
			subscriber_image = this->create_subscription<sensor_msgs::msg::Image>("camera", 10, std::bind(&MonocularSlamNode::grabImage, this, std::placeholders::_1));
		
			timer = this->create_wall_timer(100ms, std::bind(&MonocularSlamNode::loop, this));
		}

		~MonocularSlamNode()
		{
			orb_slam->Shutdown();

			// Save camera trajectory
			// orb_slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
		}

		void loop(){}

		void grabImage(const sensor_msgs::msg::Image::SharedPtr msg)
		{
			// Copy the ros image message to cv::Mat.
			try
			{
				image_ptr = cv_bridge::toCvCopy(msg);
			}
			catch (cv_bridge::Exception& e)
			{
				RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			}
			
			cv::Mat tcw = orb_slam->TrackMonocular(image_ptr->image, msg->header.stamp.sec);
		}
};


int main(int argc, char **argv)
{
	if(argc < 3)
	{
		std::cerr << "\nUsage: ros2 run orbslam2 mono <vocabulary> <config>" << std::endl;        
		return 1;
	}

	rclcpp::init(argc, argv);

	bool visualization = false;

	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, visualization);

	rclcpp::spin(std::make_shared<MonocularSlamNode>(&SLAM));
	rclcpp::shutdown();
	return 0;
}
