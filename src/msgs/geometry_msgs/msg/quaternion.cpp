#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                // RCLCPP_DEBUG
#include "geometry_msgs/msg/quaternion.hpp" // geometry_msgs::msg::Quaternion

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/quaternion.hpp" // GeometryMsgsMsgQuaternion

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgQuaternion::serialize(const std::shared_ptr<geometry_msgs::msg::Quaternion> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgQuaternion::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgQuaternion::serialize(const std::shared_ptr<geometry_msgs::msg::Quaternion> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->x), reinterpret_cast<char *>(&msg->x) + sizeof(msg->x));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::serialize"), "x: %f", msg->x);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->y), reinterpret_cast<char *>(&msg->y) + sizeof(msg->y));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::serialize"), "y: %f", msg->y);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->z), reinterpret_cast<char *>(&msg->z) + sizeof(msg->z));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::serialize"), "z: %f", msg->z);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->w), reinterpret_cast<char *>(&msg->w) + sizeof(msg->w));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::serialize"), "w: %f", msg->w);

        return packet;
    }

    geometry_msgs::msg::Quaternion GeometryMsgsMsgQuaternion::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Quaternion msg;

        GeometryMsgsMsgQuaternion::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Quaternion GeometryMsgsMsgQuaternion::deserialize(std::vector<char> &packet, geometry_msgs::msg::Quaternion &msg)
    {
        memcpy(&msg.x, packet.data(), sizeof(msg.x));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.x));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::deserialize"), "x: %f", msg.x);

        memcpy(&msg.y, packet.data(), sizeof(msg.y));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.y));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::deserialize"), "y: %f", msg.y);

        memcpy(&msg.z, packet.data(), sizeof(msg.z));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.z));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::deserialize"), "z: %f", msg.z);

        memcpy(&msg.w, packet.data(), sizeof(msg.w));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.w));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_quaternion::deserialize"), "w: %f", msg.w);

        return msg;
    }

} // namespace tcp_ip_bridge