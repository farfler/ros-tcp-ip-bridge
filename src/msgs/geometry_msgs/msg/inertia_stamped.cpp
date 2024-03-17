#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                     // RCLCPP_DEBUG
#include "geometry_msgs/msg/inertia_stamped.hpp" // geometry_msgs::msg::InertiaStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/inertia_stamped.hpp" // GeometryMsgsMsgInertiaStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"               // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/inertia.hpp"         // GeometryMsgsMsgInertia

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgInertiaStamped::serialize(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgInertiaStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgInertiaStamped::serialize(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgInertia::serialize(std::make_shared<geometry_msgs::msg::Inertia>(msg->inertia), packet);

        return packet;
    }

    geometry_msgs::msg::InertiaStamped GeometryMsgsMsgInertiaStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::InertiaStamped msg;

        GeometryMsgsMsgInertiaStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::InertiaStamped GeometryMsgsMsgInertiaStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::InertiaStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgInertia::deserialize(packet, msg.inertia);

        return msg;
    }

} // namespace tcp_ip_bridge