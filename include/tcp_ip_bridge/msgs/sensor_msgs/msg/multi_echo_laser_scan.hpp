#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__MULTI_ECHO_LASER_SCAN_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__MULTI_ECHO_LASER_SCAN_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/multi_echo_laser_scan.hpp" // sensor_msgs::msg::MultiEchoLaserScan

namespace tcp_ip_bridge
{

    class SensorMsgsMsgMultiEchoLaserScan
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::MultiEchoLaserScan deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::MultiEchoLaserScan deserialize(std::vector<char> &packet, sensor_msgs::msg::MultiEchoLaserScan &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__MULTI_ECHO_LASER_SCAN_HPP_