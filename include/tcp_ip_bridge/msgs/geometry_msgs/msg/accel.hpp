#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/accel.hpp" // geometry_msgs::msg::Accel

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgAccel
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Accel> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Accel> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Accel deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Accel deserialize(std::vector<char> &packet, geometry_msgs::msg::Accel &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_HPP_