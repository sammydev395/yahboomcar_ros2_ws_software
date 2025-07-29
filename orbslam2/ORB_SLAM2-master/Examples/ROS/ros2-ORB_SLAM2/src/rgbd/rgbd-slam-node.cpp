#include "rgbd-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(std::shared_ptr<ORB_SLAM2::System> pSLAM)
:   Node("orbslam"),
    m_SLAM(pSLAM)
{
    // Create subscribers using unique_ptr to avoid circular references
    rgb_sub = std::make_unique<message_filters::Subscriber<ImageMsg> >(static_cast<rclcpp::Node*>(this), "/color/image_raw");
    depth_sub = std::make_unique<message_filters::Subscriber<ImageMsg> >(static_cast<rclcpp::Node*>(this), "/depth/image_raw");

    syncApproximate = std::make_unique<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()  
{
    // Clean up subscribers before destruction
    syncApproximate.reset();
    rgb_sub.reset();
    depth_sub.reset();
    // SLAM system shutdown is handled in main function
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    if (m_SLAM != nullptr) {
        cv::Mat Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msgRGB->header.stamp.sec);
    }
}
