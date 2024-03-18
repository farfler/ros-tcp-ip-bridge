#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "sensor_msgs/msg/point_field.hpp" // sensor_msgs::msg::PointField

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/point_field.hpp" // SensorMsgsMsgPointField

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgPointField::serialize(const std::shared_ptr<sensor_msgs::msg::PointField> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgPointField::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgPointField::serialize(const std::shared_ptr<sensor_msgs::msg::PointField> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->name), reinterpret_cast<const char *>(&msg->name) + sizeof(msg->name));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::serialize"), "name: %s", msg->name.c_str());

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->offset), reinterpret_cast<const char *>(&msg->offset) + sizeof(msg->offset));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::serialize"), "offset: %u", msg->offset);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->datatype), reinterpret_cast<const char *>(&msg->datatype) + sizeof(msg->datatype));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::serialize"), "datatype: %u", msg->datatype);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->count), reinterpret_cast<const char *>(&msg->count) + sizeof(msg->count));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::serialize"), "count: %u", msg->count);

        return packet;
    }

    sensor_msgs::msg::PointField SensorMsgsMsgPointField::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::PointField msg;

        SensorMsgsMsgPointField::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::PointField SensorMsgsMsgPointField::deserialize(std::vector<char> &packet, sensor_msgs::msg::PointField &msg)
    {
        memcpy(&msg.name, packet.data(), sizeof(msg.name));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.name));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::deserialize"), "name: %s", msg.name.c_str());

        memcpy(&msg.offset, packet.data(), sizeof(msg.offset));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.offset));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::deserialize"), "offset: %u", msg.offset);

        memcpy(&msg.datatype, packet.data(), sizeof(msg.datatype));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.datatype));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::deserialize"), "datatype: %u", msg.datatype);

        memcpy(&msg.count, packet.data(), sizeof(msg.count));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.count));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_field::deserialize"), "count: %u", msg.count);

        return msg;
    }

} // namespace tcp_ip_bridge