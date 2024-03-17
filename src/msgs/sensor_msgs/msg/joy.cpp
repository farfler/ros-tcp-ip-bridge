#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"       // RCLCPP_DEBUG
#include "sensor_msgs/msg/joy.hpp" // sensor_msgs::msg::Joy

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/joy.hpp" // SensorMsgsMsgJoy
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp" // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgJoy::serialize(const std::shared_ptr<sensor_msgs::msg::Joy> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgJoy::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgJoy::serialize(const std::shared_ptr<sensor_msgs::msg::Joy> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        uint32_t axes_size = htonl(static_cast<uint32_t>(msg->axes.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&axes_size), reinterpret_cast<const char *>(&axes_size) + sizeof(axes_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy::serialize"), "axes_size: %u", ntohl(axes_size));

        if (msg->axes.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->axes.data()), reinterpret_cast<const char *>(msg->axes.data() + msg->axes.size()));
        }

        uint32_t buttons_size = htonl(static_cast<uint32_t>(msg->buttons.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&buttons_size), reinterpret_cast<const char *>(&buttons_size) + sizeof(buttons_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy::serialize"), "buttons_size: %u", ntohl(buttons_size));

        if (msg->buttons.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->buttons.data()), reinterpret_cast<const char *>(msg->buttons.data() + msg->buttons.size()));
        }

        return packet;
    }

    sensor_msgs::msg::Joy SensorMsgsMsgJoy::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::Joy msg;

        SensorMsgsMsgJoy::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::Joy SensorMsgsMsgJoy::deserialize(std::vector<char> &packet, sensor_msgs::msg::Joy &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        uint32_t axes_size;
        memcpy(&axes_size, packet.data(), sizeof(axes_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(axes_size));
        axes_size = ntohl(axes_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy::deserialize"), "axes_size: %u", axes_size);

        if (axes_size > 0)
        {
            msg.axes.resize(axes_size);
            memcpy(msg.axes.data(), packet.data(), axes_size * sizeof(float));
            packet.erase(packet.begin(), packet.begin() + axes_size * sizeof(float));
        }

        uint32_t buttons_size;
        memcpy(&buttons_size, packet.data(), sizeof(buttons_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(buttons_size));
        buttons_size = ntohl(buttons_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy::deserialize"), "buttons_size: %u", buttons_size);

        if (buttons_size > 0)
        {
            msg.buttons.resize(buttons_size);
            memcpy(msg.buttons.data(), packet.data(), buttons_size * sizeof(int32_t));
            packet.erase(packet.begin(), packet.begin() + buttons_size * sizeof(int32_t));
        }

        return msg;
    }

} // namespace tcp_ip_bridge