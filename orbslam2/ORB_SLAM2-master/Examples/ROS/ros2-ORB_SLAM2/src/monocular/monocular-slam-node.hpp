#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"


class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(std::shared_ptr<ORB_SLAM2::System> pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<ORB_SLAM2::System> m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

#endif
