#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_WITH_COVARIANCE_STAMPED_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_WITH_COVARIANCE_STAMPED_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp" // geometry_msgs::msg::AccelWithCovarianceStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgAccelWithCovarianceStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::AccelWithCovarianceStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::AccelWithCovarianceStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::AccelWithCovarianceStamped &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_WITH_COVARIANCE_STAMPED_HPP_