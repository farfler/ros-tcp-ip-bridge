#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                                   // RCLCPP_DEBUG
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp" // geometry_msgs::msg::TwistWithCovarianceStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist_with_covariance_stamped.hpp" // GeometryMsgsMsgTwistWithCovarianceStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                             // GeometryMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist_with_covariance.hpp"         // GeometryMsgsMsgTwistWithCovariance

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgTwistWithCovarianceStamped::serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgTwistWithCovarianceStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgTwistWithCovarianceStamped::serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgTwistWithCovariance::serialize(std::make_shared<geometry_msgs::msg::TwistWithCovariance>(msg->twist), packet);

        return packet;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped GeometryMsgsMsgTwistWithCovarianceStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::TwistWithCovarianceStamped msg;

        GeometryMsgsMsgTwistWithCovarianceStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped GeometryMsgsMsgTwistWithCovarianceStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::TwistWithCovarianceStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgTwistWithCovariance::deserialize(packet, msg.twist);

        return msg;
    }

} // namespace tcp_ip_bridge