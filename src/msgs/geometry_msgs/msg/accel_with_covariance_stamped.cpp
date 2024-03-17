#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                                   // RCLCPP_DEBUG
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp" // geometry_msgs::msg::AccelWithCovarianceStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/accel_with_covariance_stamped.hpp" // GeometryMsgsMsgAccelWithCovarianceStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                             // GeometryMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/accel_with_covariance.hpp"         // GeometryMsgsMsgAccelWithCovariance

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgAccelWithCovarianceStamped::serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgAccelWithCovarianceStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgAccelWithCovarianceStamped::serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgAccelWithCovariance::serialize(std::make_shared<geometry_msgs::msg::AccelWithCovariance>(msg->accel), packet);

        return packet;
    }

    geometry_msgs::msg::AccelWithCovarianceStamped GeometryMsgsMsgAccelWithCovarianceStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::AccelWithCovarianceStamped msg;

        GeometryMsgsMsgAccelWithCovarianceStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::AccelWithCovarianceStamped GeometryMsgsMsgAccelWithCovarianceStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::AccelWithCovarianceStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgAccelWithCovariance::deserialize(packet, msg.accel);

        return msg;
    }

} // namespace tcp_ip_bridge