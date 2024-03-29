#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__INERTIA_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__INERTIA_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/inertia.hpp" // geometry_msgs::msg::Inertia

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgInertia
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Inertia> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Inertia> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Inertia deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Inertia deserialize(std::vector<char> &packet, geometry_msgs::msg::Inertia &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__INERTIA_HPP_