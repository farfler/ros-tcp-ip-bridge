#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__QUATERNION_STAMPED_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__QUATERNION_STAMPED_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/quaternion_stamped.hpp" // geometry_msgs::msg::QuaternionStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgQuaternionStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::QuaternionStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::QuaternionStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::QuaternionStamped &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__QUATERNION_STAMPED_HPP_