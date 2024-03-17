#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                  // RCLCPP_DEBUG
#include "geometry_msgs/msg/pose_stamped.hpp" // geometry_msgs::msg::PoseStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose_stamped.hpp" // GeometryMsgsMsgPoseStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"            // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose.hpp"         // GeometryMsgsMsgPose

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPoseStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPoseStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPoseStamped::serialize(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgPose::serialize(std::make_shared<geometry_msgs::msg::Pose>(msg->pose), packet);

        return packet;
    }

    geometry_msgs::msg::PoseStamped GeometryMsgsMsgPoseStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::PoseStamped msg;

        GeometryMsgsMsgPoseStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::PoseStamped GeometryMsgsMsgPoseStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgPose::deserialize(packet, msg.pose);

        return msg;
    }

} // namespace tcp_ip_bridge