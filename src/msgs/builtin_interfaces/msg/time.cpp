#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "builtin_interfaces/msg/time.hpp" // builtin_interfaces::msg::Time

#include "tcp_ip_bridge/msgs/builtin_interfaces/msg/time.hpp" // BuiltinInterfacesMsgTime

namespace tcp_ip_bridge
{

    std::vector<char> BuiltinInterfacesMsgTime::serialize(const std::shared_ptr<builtin_interfaces::msg::Time> &msg)
    {
        std::vector<char> packet;

        BuiltinInterfacesMsgTime::serialize(msg, packet);

        return packet;
    }

    std::vector<char> BuiltinInterfacesMsgTime::serialize(const std::shared_ptr<builtin_interfaces::msg::Time> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->sec), reinterpret_cast<const char *>(&msg->sec) + sizeof(msg->sec));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'header_stamp_sec': %u", msg->sec);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->nanosec), reinterpret_cast<const char *>(&msg->nanosec) + sizeof(msg->nanosec));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'header_stamp_nanosec': %u", msg->nanosec);

        return packet;
    }

    builtin_interfaces::msg::Time BuiltinInterfacesMsgTime::deserialize(std::vector<char> &packet)
    {
        builtin_interfaces::msg::Time msg;

        BuiltinInterfacesMsgTime::deserialize(packet, msg);

        return msg;
    }

    builtin_interfaces::msg::Time BuiltinInterfacesMsgTime::deserialize(std::vector<char> &packet, builtin_interfaces::msg::Time &msg)
    {
        memcpy(&msg.sec, packet.data(), sizeof(msg.sec));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.sec));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'header_stamp_sec': %u", msg.sec);

        memcpy(&msg.nanosec, packet.data(), sizeof(msg.nanosec));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.nanosec));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'header_stamp_nanosec': %u", msg.nanosec);

        return msg;
    }

} // namespace tcp_ip_bridge