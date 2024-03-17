#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "geometry_msgs/msg/transform.hpp" // geometry_msgs::msg::Transform

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/transform.hpp"  // GeometryMsgsMsgTransform
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp"    // GeometryMsgsMsgVector3
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/quaternion.hpp" // GeometryMsgsMsgQuaternion

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgTransform::serialize(const std::shared_ptr<geometry_msgs::msg::Transform> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgTransform::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgTransform::serialize(const std::shared_ptr<geometry_msgs::msg::Transform> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->translation), packet);

        GeometryMsgsMsgQuaternion::serialize(std::make_shared<geometry_msgs::msg::Quaternion>(msg->rotation), packet);

        return packet;
    }

    geometry_msgs::msg::Transform GeometryMsgsMsgTransform::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::Transform msg;

        GeometryMsgsMsgTransform::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::Transform GeometryMsgsMsgTransform::deserialize(std::vector<char> &packet, geometry_msgs::msg::Transform &msg)
    {
        GeometryMsgsMsgVector3::deserialize(packet, msg.translation);

        GeometryMsgsMsgQuaternion::deserialize(packet, msg.rotation);

        return msg;
    }

} // namespace tcp_ip_bridge