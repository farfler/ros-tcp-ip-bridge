#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                // RCLCPP_DEBUG
#include "sensor_msgs/msg/joy_feedback.hpp" // sensor_msgs::msg::JoyFeedback

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/joy_feedback.hpp" // SensorMsgsMsgJoyFeedback

namespace tcp_ip_bridge
{
    std::vector<char> SensorMsgsMsgJoyFeedback::serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgJoyFeedback::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgJoyFeedback::serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->type), reinterpret_cast<const char *>(&msg->type) + sizeof(msg->type));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback::serialize"), "type: %d", msg->type);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->id), reinterpret_cast<const char *>(&msg->id) + sizeof(msg->id));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback::serialize"), "id: %d", msg->id);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->intensity), reinterpret_cast<const char *>(&msg->intensity) + sizeof(msg->intensity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback::serialize"), "intensity: %f", msg->intensity);

        return packet;
    }

    sensor_msgs::msg::JoyFeedback SensorMsgsMsgJoyFeedback::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::JoyFeedback msg;

        SensorMsgsMsgJoyFeedback::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::JoyFeedback SensorMsgsMsgJoyFeedback::deserialize(std::vector<char> &packet, sensor_msgs::msg::JoyFeedback &msg)
    {
        memcpy(&msg.type, packet.data(), sizeof(msg.type));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.type));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback::deserialize"), "type: %d", msg.type);

        memcpy(&msg.id, packet.data(), sizeof(msg.id));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.id));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback::deserialize"), "id: %d", msg.id);

        memcpy(&msg.intensity, packet.data(), sizeof(msg.intensity));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.intensity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joy_feedback::deserialize"), "intensity: %f", msg.intensity);

        return msg;
    }

} // namespace tcp_ip_bridge