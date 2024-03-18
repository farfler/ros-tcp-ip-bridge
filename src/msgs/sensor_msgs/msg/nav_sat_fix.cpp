#include <vector>  // std::vector
#include <cstring> // memcpy

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "sensor_msgs/msg/nav_sat_fix.hpp" // sensor_msgs::msg::NavSatFix

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/nav_sat_fix.hpp"    // SensorMsgsMsgNavSatFix
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"            // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/sensor_msgs/msg/nav_sat_status.hpp" // SensorMsgsMsgNavSatStatus

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgNavSatFix::serialize(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgNavSatFix::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgNavSatFix::serialize(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        SensorMsgsMsgNavSatStatus::serialize(std::make_shared<sensor_msgs::msg::NavSatStatus>(msg->status), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->latitude), reinterpret_cast<const char *>(&msg->latitude) + sizeof(msg->latitude));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::serialize"), "latitude: %f", msg->latitude);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->longitude), reinterpret_cast<const char *>(&msg->longitude) + sizeof(msg->longitude));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::serialize"), "longitude: %f", msg->longitude);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->altitude), reinterpret_cast<const char *>(&msg->altitude) + sizeof(msg->altitude));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::serialize"), "altitude: %f", msg->altitude);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->position_covariance), reinterpret_cast<const char *>(&msg->position_covariance) + sizeof(msg->position_covariance));

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->position_covariance_type), reinterpret_cast<const char *>(&msg->position_covariance_type) + sizeof(msg->position_covariance_type));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::serialize"), "position_covariance_type: %d", msg->position_covariance_type);

        return packet;
    }

    sensor_msgs::msg::NavSatFix SensorMsgsMsgNavSatFix::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::NavSatFix msg;

        SensorMsgsMsgNavSatFix::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::NavSatFix SensorMsgsMsgNavSatFix::deserialize(std::vector<char> &packet, sensor_msgs::msg::NavSatFix &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        SensorMsgsMsgNavSatStatus::deserialize(packet, msg.status);

        memcpy(&msg.latitude, packet.data(), sizeof(msg.latitude));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.latitude));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::deserialize"), "latitude: %f", msg.latitude);

        memcpy(&msg.longitude, packet.data(), sizeof(msg.longitude));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.longitude));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::deserialize"), "longitude: %f", msg.longitude);

        memcpy(&msg.altitude, packet.data(), sizeof(msg.altitude));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.altitude));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::deserialize"), "altitude: %f", msg.altitude);

        memcpy(&msg.position_covariance, packet.data(), sizeof(msg.position_covariance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.position_covariance));

        memcpy(&msg.position_covariance_type, packet.data(), sizeof(msg.position_covariance_type));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.position_covariance_type));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_fix::deserialize"), "position_covariance_type: %d", msg.position_covariance_type);

        return msg;
    }

} // namespace tcp_ip_bridge