#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"            // RCLCPP_DEBUG
#include "geometry_msgs/msg/wrench.hpp" // geometry_msgs::msg::Wrench

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/wrench.hpp"  // GeometryMsgsMsgWrench
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp" // GeometryMsgsMsgVector3

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgWrench::serialize(const std::shared_ptr<geometry_msgs::msg::Wrench> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgWrench::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgWrench::serialize(const std::shared_ptr<geometry_msgs::msg::Wrench> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->force), packet);

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->torque), packet);

        return packet;
    }

    geometry_msgs::msg::Wrench GeometryMsgsMsgWrench::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Wrench msg;

        GeometryMsgsMsgWrench::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Wrench GeometryMsgsMsgWrench::deserialize(std::vector<char> &packet, geometry_msgs::msg::Wrench &msg)
    {
        GeometryMsgsMsgVector3::deserialize(packet, msg.force);

        GeometryMsgsMsgVector3::deserialize(packet, msg.torque);

        return msg;
    }

} // namespace tcp_ip_bridge