#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"                      // RCLCPP_DEBUG
#include "sensor_msgs/msg/region_of_interest.hpp" // sensor_msgs::msg::RegionOfInterest

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/region_of_interest.hpp" // SensorMsgsMsgRegionOfInterest

namespace tcp_ip_bridge
{
    std::vector<char> SensorMsgsMsgRegionOfInterest::serialize(const std::shared_ptr<sensor_msgs::msg::RegionOfInterest> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgRegionOfInterest::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgRegionOfInterest::serialize(const std::shared_ptr<sensor_msgs::msg::RegionOfInterest> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->x_offset), reinterpret_cast<const char *>(&msg->x_offset) + sizeof(msg->x_offset));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::serialize"), "x_offset: %d", msg->x_offset);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->y_offset), reinterpret_cast<const char *>(&msg->y_offset) + sizeof(msg->y_offset));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::serialize"), "y_offset: %d", msg->y_offset);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->height), reinterpret_cast<const char *>(&msg->height) + sizeof(msg->height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::serialize"), "height: %d", msg->height);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->width), reinterpret_cast<const char *>(&msg->width) + sizeof(msg->width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::serialize"), "width: %d", msg->width);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->do_rectify), reinterpret_cast<const char *>(&msg->do_rectify) + sizeof(msg->do_rectify));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::serialize"), "do_rectify: %d", msg->do_rectify);

        return packet;
    }

    sensor_msgs::msg::RegionOfInterest SensorMsgsMsgRegionOfInterest::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::RegionOfInterest msg;

        SensorMsgsMsgRegionOfInterest::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::RegionOfInterest SensorMsgsMsgRegionOfInterest::deserialize(std::vector<char> &packet, sensor_msgs::msg::RegionOfInterest &msg)
    {
        memcpy(&msg.x_offset, packet.data(), sizeof(msg.x_offset));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.x_offset));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::deserialize"), "x_offset: %d", msg.x_offset);

        memcpy(&msg.y_offset, packet.data(), sizeof(msg.y_offset));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.y_offset));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::deserialize"), "y_offset: %d", msg.y_offset);

        memcpy(&msg.height, packet.data(), sizeof(msg.height));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::deserialize"), "height: %d", msg.height);

        memcpy(&msg.width, packet.data(), sizeof(msg.width));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::deserialize"), "width: %d", msg.width);

        memcpy(&msg.do_rectify, packet.data(), sizeof(msg.do_rectify));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.do_rectify));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_region_of_interest::deserialize"), "do_rectify: %d", msg.do_rectify);

        return msg;
    }

} // namespace tcp_ip_bridge