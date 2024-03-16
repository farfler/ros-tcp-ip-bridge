#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"           // RCLCPP_DEBUG
#include "std_msgs/msg/color_rgba.hpp" // std_msgs::msg::ColorRGBA

#include "tcp_ip_bridge/msgs/std_msgs/msg/color_rgba.hpp" // StdMsgsMsgColorRGBA

namespace tcp_ip_bridge
{

    std::vector<char> StdMsgsMsgColorRGBA::serialize(const std::shared_ptr<std_msgs::msg::ColorRGBA> &msg)
    {
        std::vector<char> packet;

        StdMsgsMsgColorRGBA::serialize(msg, packet);

        return packet;
    }

    std::vector<char> StdMsgsMsgColorRGBA::serialize(const std::shared_ptr<std_msgs::msg::ColorRGBA> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->r), reinterpret_cast<const char *>(&msg->r) + sizeof(msg->r));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::serialize"), "r: %f", msg->r);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->g), reinterpret_cast<const char *>(&msg->g) + sizeof(msg->g));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::serialize"), "g: %f", msg->g);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->b), reinterpret_cast<const char *>(&msg->b) + sizeof(msg->b));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::serialize"), "b: %f", msg->b);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->a), reinterpret_cast<const char *>(&msg->a) + sizeof(msg->a));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::serialize"), "a: %f", msg->a);

        return packet;
    }

    std_msgs::msg::ColorRGBA StdMsgsMsgColorRGBA::deserialize(std::vector<char> &packet)
    {
        std_msgs::msg::ColorRGBA msg;

        StdMsgsMsgColorRGBA::deserialize(packet, msg);

        return msg;
    }

    std_msgs::msg::ColorRGBA StdMsgsMsgColorRGBA::deserialize(std::vector<char> &packet, std_msgs::msg::ColorRGBA &msg)
    {
        memcpy(&msg.r, packet.data(), sizeof(msg.r));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.r));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::deserialize"), "r: %f", msg.r);

        memcpy(&msg.g, packet.data(), sizeof(msg.g));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.g));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::deserialize"), "g: %f", msg.g);

        memcpy(&msg.b, packet.data(), sizeof(msg.b));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.b));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::deserialize"), "b: %f", msg.b);

        memcpy(&msg.a, packet.data(), sizeof(msg.a));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.a));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_color_rgba::deserialize"), "a: %f", msg.a);

        return msg;
    }

} // namespace tcp_ip_bridge