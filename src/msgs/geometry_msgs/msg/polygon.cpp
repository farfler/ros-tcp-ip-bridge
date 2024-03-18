#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"             // RCLCPP_DEBUG
#include "geometry_msgs/msg/polygon.hpp" // geometry_msgs::msg::Polygon
#include "geometry_msgs/msg/point.hpp"   // geometry_msgs::msg::Point

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/polygon.hpp" // GeometryMsgsMsgPolygon

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPolygon::serialize(const std::shared_ptr<geometry_msgs::msg::Polygon> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPolygon::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPolygon::serialize(const std::shared_ptr<geometry_msgs::msg::Polygon> &msg, std::vector<char> &packet)
    {

        uint32_t points_size = htonl(static_cast<uint32_t>(msg->points.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&points_size), reinterpret_cast<const char *>(&points_size) + sizeof(points_size));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_polygon::serialize"), "points_size: %u", ntohl(points_size));

        if (msg->points.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->points.data()), reinterpret_cast<const char *>(msg->points.data() + msg->points.size() * sizeof(geometry_msgs::msg::Point)));
        }

        return packet;
    }

    geometry_msgs::msg::Polygon GeometryMsgsMsgPolygon::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Polygon msg;

        GeometryMsgsMsgPolygon::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Polygon GeometryMsgsMsgPolygon::deserialize(std::vector<char> &packet, geometry_msgs::msg::Polygon &msg)
    {
        uint32_t points_size;
        memcpy(&points_size, packet.data(), sizeof(points_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(points_size));
        points_size = ntohl(points_size);

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_polygon::deserialize"), "points_size: %u", points_size);

        if (points_size > 0)
        {
            msg.points.resize(points_size);
            memcpy(msg.points.data(), packet.data(), points_size * sizeof(geometry_msgs::msg::Point));
            packet.erase(packet.begin(), packet.begin() + points_size * sizeof(geometry_msgs::msg::Point));
        }

        return msg;
    }

} // namespace tcp_ip_bridge