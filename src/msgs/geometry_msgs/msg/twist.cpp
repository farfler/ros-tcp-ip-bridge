#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"           // RCLCPP_DEBUG
#include "geometry_msgs/msg/twist.hpp" // geometry_msgs::msg::Twist

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist.hpp"   // GeometryMsgsMsgTwist
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp" // GeometryMsgsMsgVector3

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgTwist::serialize(const std::shared_ptr<geometry_msgs::msg::Twist> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgTwist::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgTwist::serialize(const std::shared_ptr<geometry_msgs::msg::Twist> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->linear), packet);

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->angular), packet);

        return packet;
    }

    geometry_msgs::msg::Twist GeometryMsgsMsgTwist::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Twist msg;

        GeometryMsgsMsgTwist::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Twist GeometryMsgsMsgTwist::deserialize(std::vector<char> &packet, geometry_msgs::msg::Twist &msg)
    {
        GeometryMsgsMsgVector3::deserialize(packet, msg.linear);

        GeometryMsgsMsgVector3::deserialize(packet, msg.angular);

        return msg;
    }

} // namespace tcp_ip_bridge