#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                   // RCLCPP_DEBUG
#include "geometry_msgs/msg/twist_stamped.hpp" // geometry_msgs::msg::TwistStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist_stamped.hpp" // GeometryMsgsMsgTwistStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"             // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist.hpp"         // GeometryMsgsMsgTwist

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgTwistStamped::serialize(const std::shared_ptr<geometry_msgs::msg::TwistStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgTwistStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgTwistStamped::serialize(const std::shared_ptr<geometry_msgs::msg::TwistStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgTwist::serialize(std::make_shared<geometry_msgs::msg::Twist>(msg->twist), packet);

        return packet;
    }

    geometry_msgs::msg::TwistStamped GeometryMsgsMsgTwistStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::TwistStamped msg;

        GeometryMsgsMsgTwistStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::TwistStamped GeometryMsgsMsgTwistStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::TwistStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgTwist::deserialize(packet, msg.twist);

        return msg;
    }

} // namespace tcp_ip_bridge