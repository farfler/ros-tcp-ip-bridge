#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"             // RCLCPP_DEBUG
#include "geometry_msgs/msg/inertia.hpp" // geometry_msgs::msg::Inertia

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/inertia.hpp" // GeometryMsgsMsgInertia
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp" // GeometryMsgsMsgVector3

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgInertia::serialize(const std::shared_ptr<geometry_msgs::msg::Inertia> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgInertia::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgInertia::serialize(const std::shared_ptr<geometry_msgs::msg::Inertia> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->m), reinterpret_cast<char *>(&msg->m) + sizeof(msg->m));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "m: %f", msg->m);

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->com), packet);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->ixx), reinterpret_cast<char *>(&msg->ixx) + sizeof(msg->ixx));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "ixx: %f", msg->ixx);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->ixy), reinterpret_cast<char *>(&msg->ixy) + sizeof(msg->ixy));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "ixy: %f", msg->ixy);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->ixz), reinterpret_cast<char *>(&msg->ixz) + sizeof(msg->ixz));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "ixz: %f", msg->ixz);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->iyy), reinterpret_cast<char *>(&msg->iyy) + sizeof(msg->iyy));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "iyy: %f", msg->iyy);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->iyz), reinterpret_cast<char *>(&msg->iyz) + sizeof(msg->iyz));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "iyz: %f", msg->iyz);

        packet.insert(packet.end(), reinterpret_cast<char *>(&msg->izz), reinterpret_cast<char *>(&msg->izz) + sizeof(msg->izz));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::serialize"), "izz: %f", msg->izz);

        return packet;
    }

    geometry_msgs::msg::Inertia GeometryMsgsMsgInertia::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Inertia msg;

        GeometryMsgsMsgInertia::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Inertia GeometryMsgsMsgInertia::deserialize(std::vector<char> &packet, geometry_msgs::msg::Inertia &msg)
    {
        memcpy(&msg.m, packet.data(), sizeof(msg.m));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.m));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "m: %f", msg.m);

        GeometryMsgsMsgVector3::deserialize(packet, msg.com);

        memcpy(&msg.ixx, packet.data(), sizeof(msg.ixx));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.ixx));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "ixx: %f", msg.ixx);

        memcpy(&msg.ixy, packet.data(), sizeof(msg.ixy));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.ixy));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "ixy: %f", msg.ixy);

        memcpy(&msg.ixz, packet.data(), sizeof(msg.ixz));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.ixz));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "ixz: %f", msg.ixz);

        memcpy(&msg.iyy, packet.data(), sizeof(msg.iyy));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.iyy));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "iyy: %f", msg.iyy);

        memcpy(&msg.iyz, packet.data(), sizeof(msg.iyz));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.iyz));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "iyz: %f", msg.iyz);

        memcpy(&msg.izz, packet.data(), sizeof(msg.izz));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.izz));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_inertia::deserialize"), "izz: %f", msg.izz);

        return msg;
    }

} // namespace tcp_ip_bridge