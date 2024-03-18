#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_WITH_COVARIANCE_STAMPED_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_WITH_COVARIANCE_STAMPED_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // geometry_msgs::msg::PoseWithCovarianceStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPoseWithCovarianceStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::PoseWithCovarianceStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::PoseWithCovarianceStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POSE_WITH_COVARIANCE_STAMPED_HPP_