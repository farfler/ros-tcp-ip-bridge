#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_WITH_COVARIANCE_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_WITH_COVARIANCE_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/pose_with_covariance.hpp" // geometry_msgs::msg::PoseWithCovariance

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPoseWithCovariance
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovariance> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovariance> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::PoseWithCovariance deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::PoseWithCovariance deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseWithCovariance &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_WITH_COVARIANCE_HPP_