#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                  // RCLCPP_DEBUG
#include "sensor_msgs/msg/magnetic_field.hpp" // sensor_msgs::msg::MagneticField

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/magnetic_field.hpp" // SensorMsgsMsgMagneticField
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"            // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/vector3.hpp"      // GeometryMsgsMsgVector3

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgMagneticField::serialize(const std::shared_ptr<sensor_msgs::msg::MagneticField> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgMagneticField::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgMagneticField::serialize(const std::shared_ptr<sensor_msgs::msg::MagneticField> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        GeometryMsgsMsgVector3::serialize(std::make_shared<geometry_msgs::msg::Vector3>(msg->magnetic_field), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->magnetic_field_covariance), reinterpret_cast<const char *>(&msg->magnetic_field_covariance) + sizeof(msg->magnetic_field_covariance));

        return packet;
    }

    sensor_msgs::msg::MagneticField SensorMsgsMsgMagneticField::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::MagneticField msg;

        SensorMsgsMsgMagneticField::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::MagneticField SensorMsgsMsgMagneticField::deserialize(std::vector<char> &packet, sensor_msgs::msg::MagneticField &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        GeometryMsgsMsgVector3::deserialize(packet, msg.magnetic_field);

        memcpy(&msg.magnetic_field_covariance, packet.data(), sizeof(msg.magnetic_field_covariance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.magnetic_field_covariance));

        return msg;
    }

} // namespace tcp_ip_bridge