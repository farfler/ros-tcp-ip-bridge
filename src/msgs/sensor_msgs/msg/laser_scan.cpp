#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"              // RCLCPP_DEBUG
#include "sensor_msgs/msg/laser_scan.hpp" // sensor_msgs::msg::LaserScan

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/laser_scan.hpp" // SensorMsgsMsgLaserScan
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"        // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgLaserScan::serialize(const std::shared_ptr<sensor_msgs::msg::LaserScan> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgLaserScan::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgLaserScan::serialize(const std::shared_ptr<sensor_msgs::msg::LaserScan> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_min), reinterpret_cast<const char *>(&msg->angle_min) + sizeof(msg->angle_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'angle_min': %f", msg->angle_min);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_max), reinterpret_cast<const char *>(&msg->angle_max) + sizeof(msg->angle_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'angle_max': %f", msg->angle_max);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_increment), reinterpret_cast<const char *>(&msg->angle_increment) + sizeof(msg->angle_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'angle_increment': %f", msg->angle_increment);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->time_increment), reinterpret_cast<const char *>(&msg->time_increment) + sizeof(msg->time_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'time_increment': %f", msg->time_increment);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->scan_time), reinterpret_cast<const char *>(&msg->scan_time) + sizeof(msg->scan_time));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'scan_time': %f", msg->scan_time);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range_min), reinterpret_cast<const char *>(&msg->range_min) + sizeof(msg->range_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'range_min': %f", msg->range_min);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range_max), reinterpret_cast<const char *>(&msg->range_max) + sizeof(msg->range_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'range_max': %f", msg->range_max);

        uint32_t ranges_size = htonl(static_cast<uint32_t>(msg->ranges.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&ranges_size), reinterpret_cast<const char *>(&ranges_size) + sizeof(ranges_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'ranges_size': %u", msg->ranges.size());

        if (msg->ranges.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->ranges.data()), reinterpret_cast<const char *>(msg->ranges.data() + msg->ranges.size()));

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'ranges': %f", msg->ranges[0]);
        }

        uint32_t intensities_size = htonl(static_cast<uint32_t>(msg->intensities.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&intensities_size), reinterpret_cast<const char *>(&intensities_size) + sizeof(intensities_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'intensities_size': %u", msg->intensities.size());

        if (msg->intensities.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->intensities.data()), reinterpret_cast<const char *>(msg->intensities.data() + msg->intensities.size()));

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::serialize"), "Serialized 'intensities': %f", msg->intensities[0]);
        }

        return packet;
    }

    sensor_msgs::msg::LaserScan SensorMsgsMsgLaserScan::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::LaserScan msg;

        SensorMsgsMsgLaserScan::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::LaserScan SensorMsgsMsgLaserScan::deserialize(std::vector<char> &packet, sensor_msgs::msg::LaserScan &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.angle_min, packet.data(), sizeof(msg.angle_min));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angle_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'angle_min': %f", msg.angle_min);

        memcpy(&msg.angle_max, packet.data(), sizeof(msg.angle_max));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angle_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'angle_max': %f", msg.angle_max);

        memcpy(&msg.angle_increment, packet.data(), sizeof(msg.angle_increment));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.angle_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'angle_increment': %f", msg.angle_increment);

        memcpy(&msg.time_increment, packet.data(), sizeof(msg.time_increment));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.time_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'time_increment': %f", msg.time_increment);

        memcpy(&msg.scan_time, packet.data(), sizeof(msg.scan_time));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.scan_time));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'scan_time': %f", msg.scan_time);

        memcpy(&msg.range_min, packet.data(), sizeof(msg.range_min));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.range_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'range_min': %f", msg.range_min);

        memcpy(&msg.range_max, packet.data(), sizeof(msg.range_max));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.range_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'range_max': %f", msg.range_max);

        uint32_t ranges_size;
        memcpy(&ranges_size, packet.data(), sizeof(ranges_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(ranges_size));
        ranges_size = ntohl(ranges_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'ranges_size': %u", ranges_size);

        if (ranges_size > 0)
        {
            msg.ranges.resize(ranges_size);
            memcpy(msg.ranges.data(), packet.data(), ranges_size * sizeof(float));
            packet.erase(packet.begin(), packet.begin() + ranges_size * sizeof(float));

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'ranges': %u", msg.ranges.size());
        }

        uint32_t intensities_size;
        memcpy(&intensities_size, packet.data(), sizeof(intensities_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(intensities_size));
        intensities_size = ntohl(intensities_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'intensities_size': %u", intensities_size);

        if (intensities_size > 0)
        {
            msg.intensities.resize(intensities_size);
            memcpy(msg.intensities.data(), packet.data(), intensities_size * sizeof(float));
            packet.erase(packet.begin(), packet.begin() + intensities_size * sizeof(float));

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_laser_scan::deserialize"), "Deserialized 'intensities': %f", msg.intensities[0]);
        }

        return msg;
    }

} // namespace tcp_ip_bridge