#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__JOY_FEEDBACK_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__JOY_FEEDBACK_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/joy_feedback.hpp" // sensor_msgs::msg::JoyFeedback

namespace tcp_ip_bridge
{

    class SensorMsgsMsgJoyFeedback
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::JoyFeedback deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::JoyFeedback deserialize(std::vector<char> &packet, sensor_msgs::msg::JoyFeedback &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__JOY_FEEDBACK_HPP_