#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                     // RCLCPP_DEBUG
#include "geometry_msgs/msg/vector3_stamped.hpp" // geometry_msgs::msg::Vector3Stamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3_stamped.hpp" // GeometryMsgsMsgVector3Stamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"               // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp"         // GeometryMsgsMsgVector3

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgVector3Stamped::serialize(const std::shared_ptr<geometry_msgs::msg::Vector3Stamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgVector3Stamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgVector3Stamped::serialize(const std::shared_ptr<geometry_msgs::msg::Vector3Stamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->vector), packet);

        return packet;
    }

    geometry_msgs::msg::Vector3Stamped GeometryMsgsMsgVector3Stamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Vector3Stamped msg;

        GeometryMsgsMsgVector3Stamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Vector3Stamped GeometryMsgsMsgVector3Stamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::Vector3Stamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgVector3::deserialize(packet, msg.vector);

        return msg;
    }

} // namespace tcp_ip_bridge