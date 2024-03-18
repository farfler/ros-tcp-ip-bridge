#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_ARRAY_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_ARRAY_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/pose_array.hpp" // geometry_msgs::msg::PoseArray

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPoseArray
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseArray> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseArray> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::PoseArray deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::PoseArray deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseArray &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_ARRAY_HPP_