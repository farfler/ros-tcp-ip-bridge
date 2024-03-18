#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__ILLUMINANCE_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__ILLUMINANCE_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/illuminance.hpp" // sensor_msgs::msg::Illuminance

namespace tcp_ip_bridge
{

    class SensorMsgsMsgIlluminance
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Illuminance> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Illuminance> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::Illuminance deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::Illuminance deserialize(std::vector<char> &packet, sensor_msgs::msg::Illuminance &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__ILLUMINANCE_HPP_