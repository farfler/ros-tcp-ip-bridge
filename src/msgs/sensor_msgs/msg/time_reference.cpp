#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                  // RCLCPP_DEBUG
#include "sensor_msgs/msg/time_reference.hpp" // sensor_msgs::msg::TimeReference

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/time_reference.hpp" // SensorMsgsMsgTimeReference
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"            // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/builtin_interfaces/msg/time.hpp"    // BuiltinInterfacesMsgTime

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgTimeReference::serialize(const std::shared_ptr<sensor_msgs::msg::TimeReference> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgTimeReference::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgTimeReference::serialize(const std::shared_ptr<sensor_msgs::msg::TimeReference> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        BuiltinInterfacesMsgTime::serialize(std::make_shared<builtin_interfaces::msg::Time>(msg->time_ref), packet);

        uint32_t source_size = htonl(static_cast<uint32_t>(msg->source.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&source_size), reinterpret_cast<const char *>(&source_size) + sizeof(source_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_time_reference::serialize"), "source_size: %u", ntohl(source_size));

        if (msg->source.size() > 0)
        {
            packet.insert(packet.end(), msg->source.begin(), msg->source.end());

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_time_reference::serialize"), "source: %s", msg->source.c_str());
        }

        return packet;
    }

    sensor_msgs::msg::TimeReference SensorMsgsMsgTimeReference::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::TimeReference msg;

        SensorMsgsMsgTimeReference::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::TimeReference SensorMsgsMsgTimeReference::deserialize(std::vector<char> &packet, sensor_msgs::msg::TimeReference &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        BuiltinInterfacesMsgTime::deserialize(packet, msg.time_ref);

        uint32_t source_size;
        memcpy(&source_size, packet.data(), sizeof(source_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(source_size));
        source_size = ntohl(source_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_time_reference::deserialize"), "source_size: %u", source_size);

        if (source_size > 0)
        {
            msg.source.resize(source_size);
            memcpy(msg.source.data(), packet.data(), source_size);
            packet.erase(packet.begin(), packet.begin() + source_size);

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_time_reference::deserialize"), "source: %s", msg.source.c_str());
        }

        return msg;
    }

} // namespace tcp_ip_bridge