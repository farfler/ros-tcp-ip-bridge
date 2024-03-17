#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                       // RCLCPP_DEBUG
#include "geometry_msgs/msg/transform_stamped.hpp" // geometry_msgs::msg::TransformStamped

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/transform_stamped.hpp" // GeometryMsgsMsgTransformStamped
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                 // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/transform.hpp"         // GeometryMsgsMsgTransform

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgTransformStamped::serialize(const std::shared_ptr<geometry_msgs::msg::TransformStamped> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgTransformStamped::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgTransformStamped::serialize(const std::shared_ptr<geometry_msgs::msg::TransformStamped> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        uint32_t child_frame_id_size = htonl(static_cast<uint32_t>(msg->child_frame_id.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&child_frame_id_size), reinterpret_cast<const char *>(&child_frame_id_size) + sizeof(child_frame_id_size));

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::serialize"), "child_frame_id_size: %d", ntohl(child_frame_id_size));

        if (msg->child_frame_id.size() > 0)
        {
            packet.insert(packet.end(), msg->child_frame_id.data(), msg->child_frame_id.data() + msg->child_frame_id.size());

            RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::serialize"), "child_frame_id: %s", msg->child_frame_id.c_str());
        }

        GeometryMsgsMsgTransform::serialize(std::make_shared<geometry_msgs::msg::Transform>(msg->transform), packet);

        return packet;
    }

    geometry_msgs::msg::TransformStamped GeometryMsgsMsgTransformStamped::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::TransformStamped msg;

        GeometryMsgsMsgTransformStamped::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::TransformStamped GeometryMsgsMsgTransformStamped::deserialize(std::vector<char> &packet, geometry_msgs::msg::TransformStamped &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        uint32_t child_frame_id_size;
        memcpy(&child_frame_id_size, packet.data(), sizeof(child_frame_id_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(child_frame_id_size));
        child_frame_id_size = ntohl(child_frame_id_size);

        RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::deserialize"), "child_frame_id_size: %d", child_frame_id_size);

        if (child_frame_id_size > 0)
        {
            msg.child_frame_id.resize(child_frame_id_size);
            memcpy(msg.child_frame_id.data(), packet.data(), child_frame_id_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + child_frame_id_size * sizeof(char));

            RCLCPP_DEBUG(rclcpp::get_logger("std_msgs_msg_header::deserialize"), "child_frame_id: %s", msg.child_frame_id.c_str());
        }

        GeometryMsgsMsgTransform::deserialize(packet, msg.transform);

        return msg;
    }

} // namespace tcp_ip_bridge