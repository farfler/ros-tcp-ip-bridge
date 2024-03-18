#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                  // RCLCPP_DEBUG
#include "sensor_msgs/msg/nav_sat_status.hpp" // sensor_msgs::msg::NavSatStatus

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/nav_sat_status.hpp" // SensorMsgsMsgNavSatStatus

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgNavSatStatus::serialize(const std::shared_ptr<sensor_msgs::msg::NavSatStatus> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgNavSatStatus::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgNavSatStatus::serialize(const std::shared_ptr<sensor_msgs::msg::NavSatStatus> &msg, std::vector<char> &packet)
    {
        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->status), reinterpret_cast<const char *>(&msg->status) + sizeof(msg->status));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_status::serialize"), "status: %d", msg->status);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->service), reinterpret_cast<const char *>(&msg->service) + sizeof(msg->service));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_status::serialize"), "service: %d", msg->service);

        return packet;
    }

    sensor_msgs::msg::NavSatStatus SensorMsgsMsgNavSatStatus::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::NavSatStatus msg;

        SensorMsgsMsgNavSatStatus::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::NavSatStatus SensorMsgsMsgNavSatStatus::deserialize(std::vector<char> &packet, sensor_msgs::msg::NavSatStatus &msg)
    {
        memcpy(&msg.status, packet.data(), sizeof(msg.status));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.status));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_status::deserialize"), "status: %d", msg.status);

        memcpy(&msg.service, packet.data(), sizeof(msg.service));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.service));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_nav_sat_status::deserialize"), "service: %d", msg.service);

        return msg;
    }

} // namespace tcp_ip_bridge