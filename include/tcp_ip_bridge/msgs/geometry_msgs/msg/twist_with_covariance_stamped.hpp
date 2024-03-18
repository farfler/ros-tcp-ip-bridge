#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TWIST_WITH_COVARIANCE_STAMPED_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TWIST_WITH_COVARIANCE_STAMPED_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp" // geometry_msgs::msg::TwistWithCovarianceStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgTwistWithCovarianceStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::TwistWithCovarianceStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::TwistWithCovarianceStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::TwistWithCovarianceStamped &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__TWIST_WITH_COVARIANCE_STAMPED_HPP_