#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "sensor_msgs/msg/joint_state.hpp" // sensor_msgs::msg::JointState

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/joint_state.hpp" // SensorMsgsMsgJointState
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"         // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgJointState::serialize(const std::shared_ptr<sensor_msgs::msg::JointState> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgJointState::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgJointState::serialize(const std::shared_ptr<sensor_msgs::msg::JointState> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        uint32_t name_size = htonl(static_cast<uint32_t>(msg->name.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&name_size), reinterpret_cast<const char *>(&name_size) + sizeof(name_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::serialize"), "name_size: %u", ntohl(name_size));

        if (msg->name.size() > 0)
        {
            for (auto &name : msg->name)
            {
                uint32_t name_size = htonl(static_cast<uint32_t>(name.size()));
                packet.insert(packet.end(), reinterpret_cast<const char *>(&name_size), reinterpret_cast<const char *>(&name_size) + sizeof(name_size));

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::serialize"), "name_size: %u", ntohl(name_size));

                packet.insert(packet.end(), name.begin(), name.end());

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::serialize"), "name: %s", name.c_str());
            }
        }

        uint32_t position_size = htonl(static_cast<uint32_t>(msg->position.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&position_size), reinterpret_cast<const char *>(&position_size) + sizeof(position_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::serialize"), "position_size: %u", ntohl(position_size));

        if (msg->position.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->position.data()), reinterpret_cast<const char *>(msg->position.data() + msg->position.size() * sizeof(double)));
        }

        uint32_t velocity_size = htonl(static_cast<uint32_t>(msg->velocity.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&velocity_size), reinterpret_cast<const char *>(&velocity_size) + sizeof(velocity_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::serialize"), "velocity_size: %u", ntohl(velocity_size));

        if (msg->velocity.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->velocity.data()), reinterpret_cast<const char *>(msg->velocity.data() + msg->velocity.size() * sizeof(double)));
        }

        uint32_t effort_size = htonl(static_cast<uint32_t>(msg->effort.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&effort_size), reinterpret_cast<const char *>(&effort_size) + sizeof(effort_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::serialize"), "effort_size: %u", ntohl(effort_size));

        if (msg->effort.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->effort.data()), reinterpret_cast<const char *>(msg->effort.data() + msg->effort.size() * sizeof(double)));
        }

        return packet;
    }

    sensor_msgs::msg::JointState SensorMsgsMsgJointState::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::JointState msg;

        SensorMsgsMsgJointState::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::JointState SensorMsgsMsgJointState::deserialize(std::vector<char> &packet, sensor_msgs::msg::JointState &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        uint32_t name_size;
        memcpy(&name_size, packet.data(), sizeof(name_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(name_size));
        name_size = ntohl(name_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::deserialize"), "name_size: %u", name_size);

        if (name_size > 0)
        {
            msg.name.resize(name_size);
            for (auto &name : msg.name)
            {
                uint32_t name_size;
                memcpy(&name_size, packet.data(), sizeof(name_size));
                packet.erase(packet.begin(), packet.begin() + sizeof(name_size));
                name_size = ntohl(name_size);

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::deserialize"), "name_size: %u", name_size);

                name.resize(name_size);
                memcpy(name.data(), packet.data(), name_size);
                packet.erase(packet.begin(), packet.begin() + name_size);

                RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::deserialize"), "name: %s", name.c_str());
            }
        }

        uint32_t position_size;
        memcpy(&position_size, packet.data(), sizeof(position_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(position_size));
        position_size = ntohl(position_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::deserialize"), "position_size: %u", position_size);

        if (position_size > 0)
        {
            msg.position.resize(position_size);
            memcpy(msg.position.data(), packet.data(), position_size * sizeof(double));
            packet.erase(packet.begin(), packet.begin() + position_size * sizeof(double));
        }

        uint32_t velocity_size;
        memcpy(&velocity_size, packet.data(), sizeof(velocity_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(velocity_size));
        velocity_size = ntohl(velocity_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::deserialize"), "velocity_size: %u", velocity_size);

        if (velocity_size > 0)
        {
            msg.velocity.resize(velocity_size);
            memcpy(msg.velocity.data(), packet.data(), velocity_size * sizeof(double));
            packet.erase(packet.begin(), packet.begin() + velocity_size * sizeof(double));
        }

        uint32_t effort_size;
        memcpy(&effort_size, packet.data(), sizeof(effort_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(effort_size));
        effort_size = ntohl(effort_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_joint_state::deserialize"), "effort_size: %u", effort_size);

        if (effort_size > 0)
        {
            msg.effort.resize(effort_size);
            memcpy(msg.effort.data(), packet.data(), effort_size * sizeof(double));
            packet.erase(packet.begin(), packet.begin() + effort_size * sizeof(double));
        }

        return msg;
    }

} // namespace tcp_ip_bridge