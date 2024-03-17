#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"          // RCLCPP_DEBUG
#include "geometry_msgs/msg/pose.hpp" // geometry_msgs::msg::Pose

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose.hpp"       // GeometryMsgsMsgPose
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/point.hpp"      // GeometryMsgsMsgPoint
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/quaternion.hpp" // GeometryMsgsMsgQuaternion

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPose::serialize(const std::shared_ptr<geometry_msgs::msg::Pose> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPose::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPose::serialize(const std::shared_ptr<geometry_msgs::msg::Pose> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgPoint::serialize(std::make_shared<geometry_msgs::msg::Point>(msg->position), packet);

        GeometryMsgsMsgQuaternion::serialize(std::make_shared<geometry_msgs::msg::Quaternion>(msg->orientation), packet);

        return packet;
    }

    geometry_msgs::msg::Pose GeometryMsgsMsgPose::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Pose msg;

        GeometryMsgsMsgPose::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Pose GeometryMsgsMsgPose::deserialize(std::vector<char> &packet, geometry_msgs::msg::Pose &msg)
    {
        GeometryMsgsMsgPoint::deserialize(packet, msg.position);

        GeometryMsgsMsgQuaternion::deserialize(packet, msg.orientation);

        return msg;
    }

} // namespace tcp_ip_bridge