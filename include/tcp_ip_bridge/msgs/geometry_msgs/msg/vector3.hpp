#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__VECTOR3_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__VECTOR3_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/vector3.hpp" // geometry_msgs::msg::Vector3

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgVector3
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Vector3> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Vector3> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Vector3 deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Vector3 deserialize(std::vector<char> &packet, geometry_msgs::msg::Vector3 &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__VECTOR3_HPP_