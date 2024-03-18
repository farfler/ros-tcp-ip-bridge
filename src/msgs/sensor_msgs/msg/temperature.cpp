#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "sensor_msgs/msg/temperature.hpp" // sensor_msgs::msg::Temperature

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/temperature.hpp" // SensorMsgsMsgTemperature
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"         // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgTemperature::serialize(const std::shared_ptr<sensor_msgs::msg::Temperature> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgTemperature::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgTemperature::serialize(const std::shared_ptr<sensor_msgs::msg::Temperature> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->temperature), reinterpret_cast<const char *>(&msg->temperature) + sizeof(msg->temperature));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_temperature::serialize"), "temperature: %f", msg->temperature);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->variance), reinterpret_cast<const char *>(&msg->variance) + sizeof(msg->variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_temperature::serialize"), "variance: %f", msg->variance);

        return packet;
    }

    sensor_msgs::msg::Temperature SensorMsgsMsgTemperature::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::Temperature msg;

        SensorMsgsMsgTemperature::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::Temperature SensorMsgsMsgTemperature::deserialize(std::vector<char> &packet, sensor_msgs::msg::Temperature &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.temperature, packet.data(), sizeof(msg.temperature));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.temperature));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_temperature::deserialize"), "temperature: %f", msg.temperature);

        memcpy(&msg.variance, packet.data(), sizeof(msg.variance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_temperature::deserialize"), "variance: %f", msg.variance);

        return msg;
    }

} // namespace tcp_ip_bridge