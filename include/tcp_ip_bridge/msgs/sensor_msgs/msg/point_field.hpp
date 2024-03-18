#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__POINT_FIELD_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__POINT_FIELD_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/point_field.hpp" // sensor_msgs::msg::PointField

namespace tcp_ip_bridge
{

    class SensorMsgsMsgPointField
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::PointField> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::PointField> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::PointField deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::PointField deserialize(std::vector<char> &packet, sensor_msgs::msg::PointField &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__POINT_FIELD_HPP_