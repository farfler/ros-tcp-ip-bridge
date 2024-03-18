#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TRANSFORM_STAMPED_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TRANSFORM_STAMPED_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/transform_stamped.hpp" // geometry_msgs::msg::TransformStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgTransformStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TransformStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TransformStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::TransformStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::TransformStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::TransformStamped &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TRANSFORM_STAMPED_HPP_