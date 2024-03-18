#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                         // RCLCPP_DEBUG
#include "sensor_msgs/msg/multi_dof_joint_state.hpp" // sensor_msgs::msg::MultiDOFJointState

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/multi_dof_joint_state.hpp" // SensorMsgsMsgMultiDOFJointState
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                   // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgMultiDOFJointState::serialize(const std::shared_ptr<sensor_msgs::msg::MultiDOFJointState> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgMultiDOFJointState::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgMultiDOFJointState::serialize(const std::shared_ptr<sensor_msgs::msg::MultiDOFJointState> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        uint32_t joint_names_size = htonl(static_cast<uint32_t>(msg->joint_names.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&joint_names_size), reinterpret_cast<const char *>(&joint_names_size) + sizeof(joint_names_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::serialize"), "joint_names_size: %u", ntohl(joint_names_size));

        if (msg->joint_names.size() > 0)
        {
            for (auto &joint_name : msg->joint_names)
            {
                uint32_t joint_name_size = htonl(static_cast<uint32_t>(joint_name.size()));
                packet.insert(packet.end(), reinterpret_cast<const char *>(&joint_name_size), reinterpret_cast<const char *>(&joint_name_size) + sizeof(joint_name_size));

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::serialize"), "joint_name_size: %u", ntohl(joint_name_size));

                packet.insert(packet.end(), joint_name.begin(), joint_name.end());

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::serialize"), "joint_name: %s", joint_name.c_str());
            }
        }

        uint32_t transforms_size = htonl(static_cast<uint32_t>(msg->transforms.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&transforms_size), reinterpret_cast<const char *>(&transforms_size) + sizeof(transforms_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::serialize"), "transforms_size: %u", ntohl(transforms_size));

        if (msg->transforms.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->transforms.data()), reinterpret_cast<const char *>(msg->transforms.data() + msg->transforms.size() * sizeof(geometry_msgs::msg::Transform)));
        }

        uint32_t twist_size = htonl(static_cast<uint32_t>(msg->twist.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&twist_size), reinterpret_cast<const char *>(&twist_size) + sizeof(twist_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::serialize"), "twist_size: %u", ntohl(twist_size));

        if (msg->twist.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->twist.data()), reinterpret_cast<const char *>(msg->twist.data() + msg->twist.size() * sizeof(geometry_msgs::msg::Twist)));
        }

        uint32_t wrench_size = htonl(static_cast<uint32_t>(msg->wrench.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&wrench_size), reinterpret_cast<const char *>(&wrench_size) + sizeof(wrench_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::serialize"), "wrench_size: %u", ntohl(wrench_size));

        if (msg->wrench.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->wrench.data()), reinterpret_cast<const char *>(msg->wrench.data() + msg->wrench.size() * sizeof(geometry_msgs::msg::Wrench)));
        }

        return packet;
    }

    sensor_msgs::msg::MultiDOFJointState SensorMsgsMsgMultiDOFJointState::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::MultiDOFJointState msg;

        SensorMsgsMsgMultiDOFJointState::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::MultiDOFJointState SensorMsgsMsgMultiDOFJointState::deserialize(std::vector<char> &packet, sensor_msgs::msg::MultiDOFJointState &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        uint32_t joint_names_size;
        memcpy(&joint_names_size, packet.data(), sizeof(joint_names_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(joint_names_size));
        joint_names_size = ntohl(joint_names_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::deserialize"), "joint_names_size: %u", joint_names_size);

        if (joint_names_size > 0)
        {
            msg.joint_names.resize(joint_names_size);
            for (auto &joint_name : msg.joint_names)
            {
                uint32_t joint_name_size;
                memcpy(&joint_name_size, packet.data(), sizeof(joint_name_size));
                packet.erase(packet.begin(), packet.begin() + sizeof(joint_name_size));
                joint_name_size = ntohl(joint_name_size);

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::deserialize"), "joint_name_size: %u", joint_name_size);

                joint_name.resize(joint_name_size);
                memcpy(joint_name.data(), packet.data(), joint_name_size * sizeof(char));
                packet.erase(packet.begin(), packet.begin() + joint_name_size * sizeof(char));

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::deserialize"), "joint_name: %s", joint_name.c_str());
            }
        }

        uint32_t transforms_size;
        memcpy(&transforms_size, packet.data(), sizeof(transforms_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(transforms_size));
        transforms_size = ntohl(transforms_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::deserialize"), "transforms_size: %u", transforms_size);

        if (transforms_size > 0)
        {
            msg.transforms.resize(transforms_size);
            memcpy(msg.transforms.data(), packet.data(), transforms_size * sizeof(geometry_msgs::msg::Transform));
            packet.erase(packet.begin(), packet.begin() + transforms_size * sizeof(geometry_msgs::msg::Transform));
        }

        uint32_t twist_size;
        memcpy(&twist_size, packet.data(), sizeof(twist_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(twist_size));
        twist_size = ntohl(twist_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::deserialize"), "twist_size: %u", twist_size);

        if (twist_size > 0)
        {
            msg.twist.resize(twist_size);
            memcpy(msg.twist.data(), packet.data(), twist_size * sizeof(geometry_msgs::msg::Twist));
            packet.erase(packet.begin(), packet.begin() + twist_size * sizeof(geometry_msgs::msg::Twist));
        }

        uint32_t wrench_size;
        memcpy(&wrench_size, packet.data(), sizeof(wrench_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(wrench_size));
        wrench_size = ntohl(wrench_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_dof_joint_state::deserialize"), "wrench_size: %u", wrench_size);

        if (wrench_size > 0)
        {
            msg.wrench.resize(wrench_size);
            memcpy(msg.wrench.data(), packet.data(), wrench_size * sizeof(geometry_msgs::msg::Wrench));
            packet.erase(packet.begin(), packet.begin() + wrench_size * sizeof(geometry_msgs::msg::Wrench));
        }

        return msg;
    }

} // namespace tcp_ip_bridge