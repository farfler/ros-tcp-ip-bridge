#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                    // RCLCPP_DEBUG
#include "geometry_msgs/msg/wrench_stamped.hpp" // geometry_msgs::msg::WrenchStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/wrench_stamped.hpp" // GeometryMsgsMsgWrenchStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"              // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/wrench.hpp"         // GeometryMsgsMsgWrench

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgWrenchStamped::serialize(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgWrenchStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgWrenchStamped::serialize(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgWrench::serialize(std::make_shared<geometry_msgs::msg::Wrench>(msg->wrench), packet);

        return packet;
    }

    geometry_msgs::msg::WrenchStamped GeometryMsgsMsgWrenchStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::WrenchStamped msg;

        GeometryMsgsMsgWrenchStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::WrenchStamped GeometryMsgsMsgWrenchStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::WrenchStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgWrench::deserialize(packet, msg.wrench);

        return msg;
    }

} // namespace tcp_ip_bridge