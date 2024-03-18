#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"       // RCLCPP_DEBUG
#include "std_msgs/msg/header.hpp" // std_msgs::msg::Header

#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"         // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/builtin_interfaces/msg/time.hpp" // BuiltinInterfacesMsgTime

namespace tcp_ip_bridge
{

    std::vector<char> StdMsgsMsgHeader::serialize(const std::shared_ptr<std_msgs::msg::Header> &msg)
    {
        std::vector<char> packet;

        StdMsgsMsgHeader::serialize(msg, packet);

        return packet;
    }

    std::vector<char> StdMsgsMsgHeader::serialize(const std::shared_ptr<std_msgs::msg::Header> &msg, std::vector<char> &packet)
    {
        BuiltinInterfacesMsgTime::serialize(std::make_shared<builtin_interfaces::msg::Time>(msg->stamp), packet);

        uint32_t frame_id_size = htonl(static_cast<uint32_t>(msg->frame_id.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&frame_id_size), reinterpret_cast<const char *>(&frame_id_size) + sizeof(frame_id_size));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::serialize"), "frame_id_size: %u", ntohl(frame_id_size));

        if (msg->frame_id.size() > 0)
        {
            packet.insert(packet.end(), msg->frame_id.begin(), msg->frame_id.end());

            RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::serialize"), "frame_id: %s", msg->frame_id.c_str());
        }

        return packet;
    }

    std_msgs::msg::Header StdMsgsMsgHeader::deserialize(std::vector<char> &packet)
    {
        std_msgs::msg::Header msg;

        StdMsgsMsgHeader::deserialize(packet, msg);

        return msg;
    }

    std_msgs::msg::Header StdMsgsMsgHeader::deserialize(std::vector<char> &packet, std_msgs::msg::Header &msg)
    {
        BuiltinInterfacesMsgTime::deserialize(packet, msg.stamp);

        uint32_t frame_id_size;
        memcpy(&frame_id_size, packet.data(), sizeof(frame_id_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(frame_id_size));
        frame_id_size = ntohl(frame_id_size);

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::deserialize"), "frame_id_size: %d", frame_id_size);

        if (frame_id_size > 0)
        {
            msg.frame_id.resize(frame_id_size);
            memcpy(msg.frame_id.data(), packet.data(), frame_id_size);
            packet.erase(packet.begin(), packet.begin() + frame_id_size);

            RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::deserialize"), "frame_id: %s", msg.frame_id.c_str());
        }

        return msg;
    }

} // namespace tcp_ip_bridge