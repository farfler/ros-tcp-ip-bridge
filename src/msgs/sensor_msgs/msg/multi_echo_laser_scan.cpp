#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                         // RCLCPP_DEBUG
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp" // sensor_msgs::msg::MultiEchoLaserScan

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/multi_echo_laser_scan.hpp" // SensorMsgsMsgMultiEchoLaserScan
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                   // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgMultiEchoLaserScan::serialize(const std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgMultiEchoLaserScan::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgMultiEchoLaserScan::serialize(const std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_min), reinterpret_cast<const char *>(&msg->angle_min) + sizeof(msg->angle_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "angle_min: %f", msg->angle_min);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_max), reinterpret_cast<const char *>(&msg->angle_max) + sizeof(msg->angle_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "angle_max: %f", msg->angle_max);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_increment), reinterpret_cast<const char *>(&msg->angle_increment) + sizeof(msg->angle_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "angle_increment: %f", msg->angle_increment);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->time_increment), reinterpret_cast<const char *>(&msg->time_increment) + sizeof(msg->time_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "time_increment: %f", msg->time_increment);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->scan_time), reinterpret_cast<const char *>(&msg->scan_time) + sizeof(msg->scan_time));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "scan_time: %f", msg->scan_time);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range_min), reinterpret_cast<const char *>(&msg->range_min) + sizeof(msg->range_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "range_min: %f", msg->range_min);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range_max), reinterpret_cast<const char *>(&msg->range_max) + sizeof(msg->range_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "range_max: %f", msg->range_max);

        uint32_t ranges_size = htonl(static_cast<uint32_t>(msg->ranges.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&ranges_size), reinterpret_cast<const char *>(&ranges_size) + sizeof(ranges_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "ranges_size: %u", ntohl(ranges_size));

        if (msg->ranges.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->ranges.data()), reinterpret_cast<const char *>(msg->ranges.data() + msg->ranges.size() * sizeof(sensor_msgs::msg::LaserEcho)));
        }

        uint32_t intensities_size = htonl(static_cast<uint32_t>(msg->intensities.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&intensities_size), reinterpret_cast<const char *>(&intensities_size) + sizeof(intensities_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::serialize"), "intensities_size: %u", ntohl(intensities_size));

        if (msg->intensities.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->intensities.data()), reinterpret_cast<const char *>(msg->intensities.data() + msg->intensities.size() * sizeof(sensor_msgs::msg::LaserEcho)));
        }

        return packet;
    }

    sensor_msgs::msg::MultiEchoLaserScan SensorMsgsMsgMultiEchoLaserScan::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::MultiEchoLaserScan msg;

        SensorMsgsMsgMultiEchoLaserScan::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::MultiEchoLaserScan SensorMsgsMsgMultiEchoLaserScan::deserialize(std::vector<char> &packet, sensor_msgs::msg::MultiEchoLaserScan &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.angle_min, packet.data(), sizeof(msg.angle_min));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angle_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "angle_min: %f", msg.angle_min);

        memcpy(&msg.angle_max, packet.data(), sizeof(msg.angle_max));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angle_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "angle_max: %f", msg.angle_max);

        memcpy(&msg.angle_increment, packet.data(), sizeof(msg.angle_increment));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angle_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "angle_increment: %f", msg.angle_increment);

        memcpy(&msg.time_increment, packet.data(), sizeof(msg.time_increment));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.time_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "time_increment: %f", msg.time_increment);

        memcpy(&msg.scan_time, packet.data(), sizeof(msg.scan_time));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.scan_time));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "scan_time: %f", msg.scan_time);

        memcpy(&msg.range_min, packet.data(), sizeof(msg.range_min));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.range_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "range_min: %f", msg.range_min);

        memcpy(&msg.range_max, packet.data(), sizeof(msg.range_max));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.range_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "range_max: %f", msg.range_max);

        uint32_t ranges_size;
        memcpy(&ranges_size, packet.data(), sizeof(ranges_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(ranges_size));
        ranges_size = ntohl(ranges_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "ranges_size: %u", ranges_size);

        if (ranges_size > 0)
        {
            msg.ranges.resize(ranges_size);
            memcpy(msg.ranges.data(), packet.data(), ranges_size * sizeof(sensor_msgs::msg::LaserEcho));
            packet.erase(packet.begin(), packet.begin() + ranges_size * sizeof(sensor_msgs::msg::LaserEcho));
        }

        uint32_t intensities_size;
        memcpy(&intensities_size, packet.data(), sizeof(intensities_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(intensities_size));
        intensities_size = ntohl(intensities_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_multi_echo_laser_scan::deserialize"), "intensities_size: %u", intensities_size);

        if (intensities_size > 0)
        {
            msg.intensities.resize(intensities_size);
            memcpy(msg.intensities.data(), packet.data(), intensities_size * sizeof(sensor_msgs::msg::LaserEcho));
            packet.erase(packet.begin(), packet.begin() + intensities_size * sizeof(sensor_msgs::msg::LaserEcho));
        }

        return msg;
    }

} // namespace tcp_ip_bridge