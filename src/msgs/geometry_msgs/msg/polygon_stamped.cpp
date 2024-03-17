#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                     // RCLCPP_DEBUG
#include "geometry_msgs/msg/polygon_stamped.hpp" // geometry_msgs::msg::PolygonStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/polygon_stamped.hpp" // GeometryMsgsMsgPolygonStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"               // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/polygon.hpp"         // GeometryMsgsMsgPolygon

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPolygonStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PolygonStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPolygonStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPolygonStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PolygonStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgPolygon::serialize(std::make_shared<geometry_msgs::msg::Polygon>(msg->polygon), packet);

        return packet;
    }

    geometry_msgs::msg::PolygonStamped GeometryMsgsMsgPolygonStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::PolygonStamped msg;

        GeometryMsgsMsgPolygonStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::PolygonStamped GeometryMsgsMsgPolygonStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::PolygonStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgPolygon::deserialize(packet, msg.polygon);

        return msg;
    }

} // namespace tcp_ip_bridge