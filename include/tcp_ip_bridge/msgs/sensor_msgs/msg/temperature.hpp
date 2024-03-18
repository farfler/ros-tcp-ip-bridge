#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__TEMPERATURE_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__TEMPERATURE_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/temperature.hpp" // sensor_msgs::msg::Temperature

namespace tcp_ip_bridge
{

    class SensorMsgsMsgTemperature
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Temperature> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Temperature> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::Temperature deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::Temperature deserialize(std::vector<char> &packet, sensor_msgs::msg::Temperature &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__TEMPERATURE_HPP_