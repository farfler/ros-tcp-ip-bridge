#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                          // RCLCPP_DEBUG
#include "geometry_msgs/msg/pose_with_covariance.hpp" // geometry_msgs::msg::PoseWithCovariance

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose_with_covariance.hpp" // GeometryMsgsMsgPoseWithCovariance
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose.hpp"                 // GeometryMsgsMsgPose

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPoseWithCovariance::serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovariance> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPoseWithCovariance::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPoseWithCovariance::serialize(const std::shared_ptr<geometry_msgs::msg::PoseWithCovariance> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgPose::serialize(std::make_shared<geometry_msgs::msg::Pose>(msg->pose), packet);

        packet.insert(packet.end(), msg->covariance.data(), msg->covariance.data() + msg->covariance.size());

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_pose_with_covariance::serialize"), "covariance: %f", msg->covariance[0]);

        return packet;
    }

    geometry_msgs::msg::PoseWithCovariance GeometryMsgsMsgPoseWithCovariance::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::PoseWithCovariance msg;

        GeometryMsgsMsgPoseWithCovariance::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::PoseWithCovariance GeometryMsgsMsgPoseWithCovariance::deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseWithCovariance &msg)
    {
        GeometryMsgsMsgPose::deserialize(packet, msg.pose);

        memcpy(msg.covariance.data(), packet.data(), msg.covariance.size() * sizeof(double));
        packet.erase(packet.begin(), packet.begin() + msg.covariance.size() * sizeof(double));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_pose_with_covariance::deserialize"), "covariance: %f", msg.covariance[0]);

        return msg;
    }

} // namespace tcp_ip_bridge