#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"       // RCLCPP_DEBUG
#include "sensor_msgs/msg/imu.hpp" // sensor_msgs::msg::Imu

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/imu.hpp"          // SensorMsgsMsgImu
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"          // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/quaternion.hpp" // GeometryMsgsMsgQuaternion
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp"    // GeometryMsgsMsgVector3

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgImu::serialize(const std::shared_ptr<sensor_msgs::msg::Imu> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgImu::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgImu::serialize(const std::shared_ptr<sensor_msgs::msg::Imu> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgQuaternion::serialize(std::make_shared<geometry_msgs::msg::Quaternion>(msg->orientation), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->orientation_covariance), reinterpret_cast<const char *>(&msg->orientation_covariance) + sizeof(msg->orientation_covariance));

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->angular_velocity), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angular_velocity_covariance), reinterpret_cast<const char *>(&msg->angular_velocity_covariance) + sizeof(msg->angular_velocity_covariance));

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->linear_acceleration), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->linear_acceleration_covariance), reinterpret_cast<const char *>(&msg->linear_acceleration_covariance) + sizeof(msg->linear_acceleration_covariance));

        return packet;
    }

    sensor_msgs::msg::Imu SensorMsgsMsgImu::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::Imu msg;

        SensorMsgsMsgImu::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::Imu SensorMsgsMsgImu::deserialize(std::vector<char> &packet, sensor_msgs::msg::Imu &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgQuaternion::deserialize(packet, msg.orientation);

        memcpy(&msg.orientation_covariance, packet.data(), sizeof(msg.orientation_covariance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.orientation_covariance));

        GeometryMsgsMsgVector3::deserialize(packet, msg.angular_velocity);

        memcpy(&msg.angular_velocity_covariance, packet.data(), sizeof(msg.angular_velocity_covariance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angular_velocity_covariance));

        GeometryMsgsMsgVector3::deserialize(packet, msg.linear_acceleration);

        memcpy(&msg.linear_acceleration_covariance, packet.data(), sizeof(msg.linear_acceleration_covariance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.linear_acceleration_covariance));

        return msg;
    }

} // namespace tcp_ip_bridge