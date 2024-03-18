#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"              // RCLCPP_DEBUG
#include "sensor_msgs/msg/laser_echo.hpp" // sensor_msgs::msg::LaserEcho

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/laser_echo.hpp" // SensorMsgsMsgLaserEcho
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"        // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgLaserEcho::serialize(const std::shared_ptr<sensor_msgs::msg::LaserEcho> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgLaserEcho::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgLaserEcho::serialize(const std::shared_ptr<sensor_msgs::msg::LaserEcho> &msg, std::vector<char> &packet)
    {
        uint32_t echoes_size = htonl(static_cast<uint32_t>(msg->echoes.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&echoes_size), reinterpret_cast<const char *>(&echoes_size) + sizeof(echoes_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_echo::serialize"), "echoes_size: %u", ntohl(echoes_size));

        if (msg->echoes.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->echoes.data()), reinterpret_cast<const char *>(msg->echoes.data() + msg->echoes.size()));
        }

        return packet;
    }

    sensor_msgs::msg::LaserEcho SensorMsgsMsgLaserEcho::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::LaserEcho msg;

        SensorMsgsMsgLaserEcho::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::LaserEcho SensorMsgsMsgLaserEcho::deserialize(std::vector<char> &packet, sensor_msgs::msg::LaserEcho &msg)
    {
        uint32_t echoes_size;
        memcpy(&echoes_size, packet.data(), sizeof(echoes_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(echoes_size));
        echoes_size = ntohl(echoes_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_echo::deserialize"), "echoes_size: %u", echoes_size);

        if (echoes_size > 0)
        {
            msg.echoes.resize(echoes_size);
            memcpy(msg.echoes.data(), packet.data(), echoes_size * sizeof(float));
            packet.erase(packet.begin(), packet.begin() + echoes_size * sizeof(float));
        }

        return msg;
    }

} // namespace tcp_ip_bridge