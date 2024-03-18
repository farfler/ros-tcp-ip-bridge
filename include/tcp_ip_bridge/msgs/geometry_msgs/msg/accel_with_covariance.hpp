#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_WITH_COVARIANCE_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_WITH_COVARIANCE_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/accel_with_covariance.hpp" // geometry_msgs::msg::AccelWithCovariance

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgAccelWithCovariance
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovariance> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovariance> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::AccelWithCovariance deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::AccelWithCovariance deserialize(std::vector<char> &packet, geometry_msgs::msg::AccelWithCovariance &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__ACCEL_WITH_COVARIANCE_HPP_