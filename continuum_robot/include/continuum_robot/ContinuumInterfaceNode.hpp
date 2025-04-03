#ifndef CONTINUUM_INTERFACE_NODE_HPP_
#define CONTINUUM_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class Continuum;

class ContinuumInterfaceNode : public rclcpp::Node
{
public:
    ContinuumInterfaceNode();

private:
    void publish_tf();

    std::shared_ptr<Continuum> robot_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // CONTINUUM_INTERFACE_NODE_HPP_
