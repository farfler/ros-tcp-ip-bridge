#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                      // RCLCPP_DEBUG
#include "sensor_msgs/msg/joy_feedback_array.hpp" // sensor_msgs::msg::JoyFeedbackArray

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/joy_feedback_array.hpp" // SensorMsgsMsgJoyFeedbackArray

namespace tcp_ip_bridge
{
    std::vector<char> SensorMsgsMsgJoyFeedbackArray::serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgJoyFeedbackArray::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgJoyFeedbackArray::serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> &msg, std::vector<char> &packet)
    {
        uint32_t array_size = htonl(static_cast<uint32_t>(msg->array.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&array_size), reinterpret_cast<const char *>(&array_size) + sizeof(array_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback_array::serialize"), "array_size: %u", ntohl(array_size));

        if (msg->array.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->array.data()), reinterpret_cast<const char *>(msg->array.data() + msg->array.size() * sizeof(sensor_msgs::msg::JoyFeedback)));
        }

        return packet;
    }

    sensor_msgs::msg::JoyFeedbackArray SensorMsgsMsgJoyFeedbackArray::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::JoyFeedbackArray msg;

        SensorMsgsMsgJoyFeedbackArray::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::JoyFeedbackArray SensorMsgsMsgJoyFeedbackArray::deserialize(std::vector<char> &packet, sensor_msgs::msg::JoyFeedbackArray &msg)
    {
        uint32_t array_size;
        memcpy(&array_size, packet.data(), sizeof(array_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(array_size));
        array_size = ntohl(array_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback_array::deserialize"), "array_size: %u", array_size);

        if (array_size > 0)
        {
            msg.array.resize(array_size);
            memcpy(msg.array.data(), packet.data(), array_size * sizeof(sensor_msgs::msg::JoyFeedback));
            packet.erase(packet.begin(), packet.begin() + array_size * sizeof(sensor_msgs::msg::JoyFeedback));
        }

        return msg;
    }

} // namespace tcp_ip_bridge