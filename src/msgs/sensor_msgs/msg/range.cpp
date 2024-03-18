#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"         // RCLCPP_DEBUG
#include "sensor_msgs/msg/range.hpp" // sensor_msgs::msg::Range

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/range.hpp" // SensorMsgsMsgRange
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"   // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgRange::serialize(const std::shared_ptr<sensor_msgs::msg::Range> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgRange::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgRange::serialize(const std::shared_ptr<sensor_msgs::msg::Range> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->radiation_type), reinterpret_cast<const char *>(&msg->radiation_type) + sizeof(msg->radiation_type));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::serialize"), "radiation_type: %d", msg->radiation_type);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->field_of_view), reinterpret_cast<const char *>(&msg->field_of_view) + sizeof(msg->field_of_view));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::serialize"), "field_of_view: %f", msg->field_of_view);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->min_range), reinterpret_cast<const char *>(&msg->min_range) + sizeof(msg->min_range));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::serialize"), "min_range: %f", msg->min_range);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->max_range), reinterpret_cast<const char *>(&msg->max_range) + sizeof(msg->max_range));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::serialize"), "max_range: %f", msg->max_range);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range), reinterpret_cast<const char *>(&msg->range) + sizeof(msg->range));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::serialize"), "range: %f", msg->range);

        return packet;
    }

    sensor_msgs::msg::Range SensorMsgsMsgRange::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::Range msg;

        SensorMsgsMsgRange::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::Range SensorMsgsMsgRange::deserialize(std::vector<char> &packet, sensor_msgs::msg::Range &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.radiation_type, packet.data(), sizeof(msg.radiation_type));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.radiation_type));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::deserialize"), "radiation_type: %d", msg.radiation_type);

        memcpy(&msg.field_of_view, packet.data(), sizeof(msg.field_of_view));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.field_of_view));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::deserialize"), "field_of_view: %f", msg.field_of_view);

        memcpy(&msg.min_range, packet.data(), sizeof(msg.min_range));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.min_range));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::deserialize"), "min_range: %f", msg.min_range);

        memcpy(&msg.max_range, packet.data(), sizeof(msg.max_range));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.max_range));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::deserialize"), "max_range: %f", msg.max_range);

        memcpy(&msg.range, packet.data(), sizeof(msg.range));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.range));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_range::deserialize"), "range: %f", msg.range);

        return msg;
    }

} // namespace tcp_ip_bridge