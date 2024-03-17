#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"             // RCLCPP_DEBUG
#include "geometry_msgs/msg/point32.hpp" // geometry_msgs::msg::Point32

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/point32.hpp" // GeometryMsgsMsgPoint32

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPoint32::serialize(const std::shared_ptr<geometry_msgs::msg::Point32> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPoint32::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPoint32::serialize(const std::shared_ptr<geometry_msgs::msg::Point32> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->x), reinterpret_cast<const char *>(&msg->x) + sizeof(msg->x));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_point32::serialize"), "x: %f", msg->x);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->y), reinterpret_cast<const char *>(&msg->y) + sizeof(msg->y));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_point32::serialize"), "y: %f", msg->y);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->z), reinterpret_cast<const char *>(&msg->z) + sizeof(msg->z));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_point32::serialize"), "z: %f", msg->z);

        return packet;
    }

    geometry_msgs::msg::Point32 GeometryMsgsMsgPoint32::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Point32 msg;

        GeometryMsgsMsgPoint32::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Point32 GeometryMsgsMsgPoint32::deserialize(std::vector<char> &packet, geometry_msgs::msg::Point32 &msg)
    {
        memcpy(&msg.x, packet.data(), sizeof(msg.x));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.x));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_point32::deserialize"), "x: %f", msg.x);

        memcpy(&msg.y, packet.data(), sizeof(msg.y));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.y));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_point32::deserialize"), "y: %f", msg.y);

        memcpy(&msg.z, packet.data(), sizeof(msg.z));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.z));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_point32::deserialize"), "z: %f", msg.z);

        return msg;
    }

} // namespace tcp_ip_bridge