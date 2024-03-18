#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__LASER_SCAN_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__LASER_SCAN_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/laser_scan.hpp" // sensor_msgs::msg::LaserScan

namespace tcp_ip_bridge
{

    class SensorMsgsMsgLaserScan
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::LaserScan> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::LaserScan> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::LaserScan deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::LaserScan deserialize(std::vector<char> &packet, sensor_msgs::msg::LaserScan &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__LASER_SCAN_HPP_