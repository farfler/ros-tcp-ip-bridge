#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__MAGNETIC_FIELD_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__MAGNETIC_FIELD_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/magnetic_field.hpp" // sensor_msgs::msg::MagneticField

namespace tcp_ip_bridge
{

    class SensorMsgsMsgMagneticField
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::MagneticField> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::MagneticField> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::MagneticField deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::MagneticField deserialize(std::vector<char> &packet, sensor_msgs::msg::MagneticField &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__MAGNETIC_FIELD_HPP_