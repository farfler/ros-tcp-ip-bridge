#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__LASER_ECHO_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__LASER_ECHO_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/laser_echo.hpp" // sensor_msgs::msg::LaserEcho

namespace tcp_ip_bridge
{

    class SensorMsgsMsgLaserEcho
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::LaserEcho> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::LaserEcho> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::LaserEcho deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::LaserEcho deserialize(std::vector<char> &packet, sensor_msgs::msg::LaserEcho &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__LASER_ECHO_HPP_