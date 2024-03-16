#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                   // RCLCPP_DEBUG
#include "geometry_msgs/msg/point_stamped.hpp" // geometry_msgs::msg::PointStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/point_stamped.hpp" // GeometryMsgsMsgPointStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"             // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/point.hpp"         // GeometryMsgsMsgPoint

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPointStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PointStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPointStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPointStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PointStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgPoint::serialize(std::make_shared<geometry_msgs::msg::Point>(msg->point), packet);

        return packet;
    }

    geometry_msgs::msg::PointStamped GeometryMsgsMsgPointStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::PointStamped msg;

        GeometryMsgsMsgPointStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::PointStamped GeometryMsgsMsgPointStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::PointStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgPoint::deserialize(packet, msg.point);

        return msg;
    }

} // namespace tcp_ip_bridge