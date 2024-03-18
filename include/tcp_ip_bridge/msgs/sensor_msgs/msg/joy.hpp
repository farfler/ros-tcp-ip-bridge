#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__JOY_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__JOY_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/joy.hpp" // sensor_msgs::msg::Joy

namespace tcp_ip_bridge
{

    class SensorMsgsMsgJoy
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Joy> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Joy> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::Joy deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::Joy deserialize(std::vector<char> &packet, sensor_msgs::msg::Joy &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__JOY_HPP_