#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TWIST_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TWIST_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/twist.hpp" // geometry_msgs::msg::Twist

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgTwist
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Twist> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Twist> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Twist deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Twist deserialize(std::vector<char> &packet, geometry_msgs::msg::Twist &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TWIST_HPP_