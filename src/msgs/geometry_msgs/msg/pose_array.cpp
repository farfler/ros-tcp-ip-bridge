#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                // RCLCPP_DEBUG
#include "geometry_msgs/msg/pose_array.hpp" // geometry_msgs::msg::PoseArray

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/pose_array.hpp" // GeometryMsgsMsgPoseArray
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"          // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgPoseArray::serialize(const std::shared_ptr<geometry_msgs::msg::PoseArray> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgPoseArray::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgPoseArray::serialize(const std::shared_ptr<geometry_msgs::msg::PoseArray> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        uint32_t poses_size = htonl(static_cast<uint32_t>(msg->poses.size()));
        packet.insert(packet.end(), reinterpret_cast<char *>(&poses_size), reinterpret_cast<char *>(&poses_size) + sizeof(poses_size));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_pose_array::serialize"), "poses_size: %u", ntohl(poses_size));

        if (msg->poses.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->poses.data()), reinterpret_cast<const char *>(msg->poses.data() + msg->poses.size() * sizeof(geometry_msgs::msg::Pose)));
        }

        return packet;
    }

    geometry_msgs::msg::PoseArray GeometryMsgsMsgPoseArray::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::PoseArray msg;

        GeometryMsgsMsgPoseArray::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::PoseArray GeometryMsgsMsgPoseArray::deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseArray &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        uint32_t poses_size;
        memcpy(&poses_size, packet.data(), sizeof(poses_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(poses_size));
        poses_size = ntohl(poses_size);

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_pose_array::deserialize"), "poses_size: %u", poses_size);

        if (poses_size > 0)
        {
            msg.poses.resize(poses_size);
            memcpy(msg.poses.data(), packet.data(), poses_size * sizeof(geometry_msgs::msg::Pose));
            packet.erase(packet.begin(), packet.begin() + poses_size * sizeof(geometry_msgs::msg::Pose));
        }

        return msg;
    }

} // namespace tcp_ip_bridge