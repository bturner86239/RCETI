#include "rceti_continuum/ContinuumInterfaceNode.hpp"
#include "rceti_continuum/Continuum.h"

using namespace std::chrono_literals;

ContinuumInterfaceNode::ContinuumInterfaceNode()
: Node("continuum_interface_node")
{
    RCLCPP_INFO(this->get_logger(), "Continuum Interface Node created (waiting to init robot)...");

    robot_ = std::make_shared<Continuum>(shared_from_this());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&ContinuumInterfaceNode::publish_tf, this)
    );
}

void ContinuumInterfaceNode::publish_tf()
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "world";  // or "map", depending on your URDF setup
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContinuumInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
