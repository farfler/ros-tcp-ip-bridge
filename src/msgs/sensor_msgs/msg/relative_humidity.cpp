#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                     // RCLCPP_DEBUG
#include "sensor_msgs/msg/relative_humidity.hpp" // sensor_msgs::msg::RelativeHumidity

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/relative_humidity.hpp" // SensorMsgsMsgRelativeHumidity
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"               // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgRelativeHumidity::serialize(const std::shared_ptr<sensor_msgs::msg::RelativeHumidity> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgRelativeHumidity::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgRelativeHumidity::serialize(const std::shared_ptr<sensor_msgs::msg::RelativeHumidity> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->relative_humidity), reinterpret_cast<const char *>(&msg->relative_humidity) + sizeof(msg->relative_humidity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_relative_humidity::serialize"), "relative_humidity: %f", msg->relative_humidity);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->variance), reinterpret_cast<const char *>(&msg->variance) + sizeof(msg->variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_relative_humidity::serialize"), "variance: %f", msg->variance);

        return packet;
    }

    sensor_msgs::msg::RelativeHumidity SensorMsgsMsgRelativeHumidity::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::RelativeHumidity msg;

        SensorMsgsMsgRelativeHumidity::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::RelativeHumidity SensorMsgsMsgRelativeHumidity::deserialize(std::vector<char> &packet, sensor_msgs::msg::RelativeHumidity &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.relative_humidity, packet.data(), sizeof(msg.relative_humidity));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.relative_humidity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_relative_humidity::deserialize"), "relative_humidity: %f", msg.relative_humidity);

        memcpy(&msg.variance, packet.data(), sizeof(msg.variance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_relative_humidity::deserialize"), "variance: %f", msg.variance);

        return msg;
    }

} // namespace tcp_ip_bridge