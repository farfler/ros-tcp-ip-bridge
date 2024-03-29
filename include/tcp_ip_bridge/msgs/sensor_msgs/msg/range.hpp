#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__RANGE_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__RANGE_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/range.hpp" // sensor_msgs::msg::Range

namespace tcp_ip_bridge
{

    class SensorMsgsMsgRange
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Range> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Range> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::Range deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::Range deserialize(std::vector<char> &packet, sensor_msgs::msg::Range &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__RANGE_HPP_