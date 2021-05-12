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
        using ImageMsg = sensor_msgs::msg::Image;

        ORB_SLAM2::System* m_SLAM;

        cv_bridge::CvImagePtr m_cvImPtr;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

        MonocularSlamNode(ORB_SLAM2::System* pSLAM): Node("orbslam"), m_SLAM(pSLAM)
        {
            m_image_subscriber = this->create_subscription<ImageMsg>("camera", 10, std::bind(&MonocularSlamNode::grabImage, this, std::placeholders::_1));
        }

        ~MonocularSlamNode()
        {
            // Stop all threads
            m_SLAM->Shutdown();

            // Save camera trajectory
            m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        }


        void grabImage(const ImageMsg::SharedPtr msg)
        {
            // Copy the ros image message to cv::Mat.
            try
            {
                m_cvImPtr = cv_bridge::toCvCopy(msg);
            }
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            
            cv::Mat Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);
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

    bool visualization = true;

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, visualization);

    rclcpp::spin(std::make_shared<MonocularSlamNode>(&SLAM));
    rclcpp::shutdown();
    return 0;
}
