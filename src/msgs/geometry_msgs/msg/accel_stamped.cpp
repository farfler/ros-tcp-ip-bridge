#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                   // RCLCPP_DEBUG
#include "geometry_msgs/msg/accel_stamped.hpp" // geometry_msgs::msg::AccelStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/accel_stamped.hpp" // GeometryMsgsMsgAccelStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"             // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/accel.hpp"         // GeometryMsgsMsgAccel

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgAccelStamped::serialize(const std::shared_ptr<geometry_msgs::msg::AccelStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgAccelStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgAccelStamped::serialize(const std::shared_ptr<geometry_msgs::msg::AccelStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgAccel::serialize(std::make_shared<geometry_msgs::msg::Accel>(msg->accel), packet);

        return packet;
    }

    geometry_msgs::msg::AccelStamped GeometryMsgsMsgAccelStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::AccelStamped msg;

        GeometryMsgsMsgAccelStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::AccelStamped GeometryMsgsMsgAccelStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::AccelStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgAccel::deserialize(packet, msg.accel);

        return msg;
    }

} // namespace tcp_ip_bridge