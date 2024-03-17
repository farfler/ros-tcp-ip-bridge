#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                        // RCLCPP_DEBUG
#include "geometry_msgs/msg/quaternion_stamped.hpp" // geometry_msgs::msg::QuaternionStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/quaternion_stamped.hpp" // GeometryMsgsMsgQuaternionStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                  // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/quaternion.hpp"         // GeometryMsgsMsgQuaternion

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgQuaternionStamped::serialize(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgQuaternionStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgQuaternionStamped::serialize(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgQuaternion::serialize(std::make_shared<geometry_msgs::msg::Quaternion>(msg->quaternion), packet);

        return packet;
    }

    geometry_msgs::msg::QuaternionStamped GeometryMsgsMsgQuaternionStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::QuaternionStamped msg;

        GeometryMsgsMsgQuaternionStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::QuaternionStamped GeometryMsgsMsgQuaternionStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::QuaternionStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgQuaternion::deserialize(packet, msg.quaternion);

        return msg;
    }

} // namespace tcp_ip_bridge