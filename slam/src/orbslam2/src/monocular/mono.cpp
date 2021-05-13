#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include"ORB_SLAM2/System.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR);
    auto node = std::make_shared<MonocularSlamNode>(&SLAM, argv[1], argv[2]);

    rclcpp::spin(node);
    

    rclcpp::shutdown();

    return 0;
}




