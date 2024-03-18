#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__QUATERNION_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__QUATERNION_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/quaternion.hpp" // geometry_msgs::msg::Quaternion

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgQuaternion
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Quaternion> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Quaternion> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Quaternion deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Quaternion deserialize(std::vector<char> &packet, geometry_msgs::msg::Quaternion &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__QUATERNION_HPP_