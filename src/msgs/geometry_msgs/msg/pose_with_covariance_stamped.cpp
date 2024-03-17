#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                                  // RCLCPP_DEBUG
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // geometry_msgs::msg::PoseWithCovarianceStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose_with_covariance_stamped.hpp" // GeometryMsgsMsgPoseWithCovarianceStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                            // GeometryMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose_with_covariance.hpp"         // GeometryMsgsMsgPoseWithCovariance

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPoseWithCovarianceStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPoseWithCovarianceStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPoseWithCovarianceStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgPoseWithCovariance::serialize(std::make_shared<geometry_msgs::msg::PoseWithCovariance>(msg->pose), packet);

        return packet;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped GeometryMsgsMsgPoseWithCovarianceStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;

        GeometryMsgsMsgPoseWithCovarianceStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped GeometryMsgsMsgPoseWithCovarianceStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgPoseWithCovariance::deserialize(packet, msg.pose);

        return msg;
    }

} // namespace tcp_ip_bridge