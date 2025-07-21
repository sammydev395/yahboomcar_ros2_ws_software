#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include"System.h"


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;        
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    auto slamSystem = std::make_shared<ORB_SLAM2::System>(
        argv[1],
        argv[2],
        ORB_SLAM2::System::MONOCULAR,
        visualization);

    auto node = std::make_shared<MonocularSlamNode>(slamSystem);

    rclcpp::spin(node);
    slamSystem->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}
