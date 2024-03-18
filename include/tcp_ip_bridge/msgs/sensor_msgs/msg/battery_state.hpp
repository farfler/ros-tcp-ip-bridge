#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__BATTERY_STATE_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__BATTERY_STATE_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/battery_state.hpp" // sensor_msgs::msg::BatteryState

namespace tcp_ip_bridge
{

    class SensorMsgsMsgBatteryState
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::BatteryState> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::BatteryState> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::BatteryState deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::BatteryState deserialize(std::vector<char> &packet, sensor_msgs::msg::BatteryState &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__BATTERY_STATE_HPP_