#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "sensor_msgs/msg/illuminance.hpp" // sensor_msgs::msg::Illuminance

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/illuminance.hpp" // SensorMsgsMsgIlluminance
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"         // StdMsgsMsgHeader

namespace tcp_ip_bridge
{
    std::vector<char> SensorMsgsMsgIlluminance::serialize(const std::shared_ptr<sensor_msgs::msg::Illuminance> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgIlluminance::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgIlluminance::serialize(const std::shared_ptr<sensor_msgs::msg::Illuminance> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->illuminance), reinterpret_cast<const char *>(&msg->illuminance) + sizeof(msg->illuminance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_illuminance::serialize"), "illuminance: %f", msg->illuminance);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->variance), reinterpret_cast<const char *>(&msg->variance) + sizeof(msg->variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_illuminance::serialize"), "variance: %f", msg->variance);

        return packet;
    }

    sensor_msgs::msg::Illuminance SensorMsgsMsgIlluminance::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::Illuminance msg;

        SensorMsgsMsgIlluminance::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::Illuminance SensorMsgsMsgIlluminance::deserialize(std::vector<char> &packet, sensor_msgs::msg::Illuminance &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.illuminance, packet.data(), sizeof(msg.illuminance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.illuminance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_illuminance::deserialize"), "illuminance: %f", msg.illuminance);

        memcpy(&msg.variance, packet.data(), sizeof(msg.variance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_illuminance::deserialize"), "variance: %f", msg.variance);

        return msg;
    }

} // namespace tcp_ip_bridge