#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                   // RCLCPP_DEBUG
#include "builtin_interfaces/msg/duration.hpp" // builtin_interfaces::msg::Duration

#include "tcp_ip_bridge/msgs/builtin_interfaces/msg/duration.hpp" // BuiltinInterfacesMsgDuration

namespace tcp_ip_bridge
{

    std::vector<char> BuiltinInterfacesMsgDuration::serialize(const std::shared_ptr<builtin_interfaces::msg::Duration> &msg)
    {
        std::vector<char> packet;

        BuiltinInterfacesMsgDuration::serialize(msg, packet);

        return packet;
    }

    std::vector<char> BuiltinInterfacesMsgDuration::serialize(const std::shared_ptr<builtin_interfaces::msg::Duration> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->sec), reinterpret_cast<const char *>(&msg->sec) + sizeof(msg->sec));

        RCLCPP_DEBUG(rclcpp::get_logger("builtin_interfaces_msg_duration::serialize"), "sec: %d", msg->sec);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->nanosec), reinterpret_cast<const char *>(&msg->nanosec) + sizeof(msg->nanosec));

        RCLCPP_DEBUG(rclcpp::get_logger("builtin_interfaces_msg_duration::serialize"), "nanosec: %u", msg->nanosec);

        return packet;
    }

    builtin_interfaces::msg::Duration BuiltinInterfacesMsgDuration::deserialize(std::vector<char> &packet)
    {
        builtin_interfaces::msg::Duration msg;

        BuiltinInterfacesMsgDuration::deserialize(packet, msg);

        return msg;
    }

    builtin_interfaces::msg::Duration BuiltinInterfacesMsgDuration::deserialize(std::vector<char> &packet, builtin_interfaces::msg::Duration &msg)
    {
        memcpy(&msg.sec, packet.data(), sizeof(msg.sec));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.sec));

        RCLCPP_DEBUG(rclcpp::get_logger("builtin_interfaces_msg_duration::deserialize"), "sec: %d", msg.sec);

        memcpy(&msg.nanosec, packet.data(), sizeof(msg.nanosec));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.nanosec));

        RCLCPP_DEBUG(rclcpp::get_logger("builtin_interfaces_msg_duration::deserialize"), "nanosec: %u", msg.nanosec);

        return msg;
    }

} // namespace tcp_ip_bridge