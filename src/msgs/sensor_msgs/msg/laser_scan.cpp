#include <vector>
#include <iostream>
#include <cstring>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/laser_scan.hpp"

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgLaserScan::serialize(const std::shared_ptr<sensor_msgs::msg::LaserScan> &msg)
    {
        std::vector<char> packet;

        uint32_t header_frame_id_size = htonl(static_cast<uint32_t>(msg->header.frame_id.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&header_frame_id_size), reinterpret_cast<const char *>(&header_frame_id_size) + sizeof(header_frame_id_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "header_frame_id_size: %u", header_frame_id_size);

        packet.insert(packet.end(), msg->header.frame_id.begin(), msg->header.frame_id.end());

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "header_frame_id: %s", msg->header.frame_id.c_str());

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->header.stamp.sec), reinterpret_cast<const char *>(&msg->header.stamp.sec) + sizeof(msg->header.stamp.sec));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "header_stamp_sec: %u", msg->header.stamp.sec);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->header.stamp.nanosec), reinterpret_cast<const char *>(&msg->header.stamp.nanosec) + sizeof(msg->header.stamp.nanosec));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "header_stamp_nanosec: %u", msg->header.stamp.nanosec);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_min), reinterpret_cast<const char *>(&msg->angle_min) + sizeof(msg->angle_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "angle_min: %f", msg->angle_min);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_max), reinterpret_cast<const char *>(&msg->angle_max) + sizeof(msg->angle_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "angle_max: %f", msg->angle_max);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->angle_increment), reinterpret_cast<const char *>(&msg->angle_increment) + sizeof(msg->angle_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "angle_increment: %f", msg->angle_increment);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->time_increment), reinterpret_cast<const char *>(&msg->time_increment) + sizeof(msg->time_increment));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "time_increment: %f", msg->time_increment);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->scan_time), reinterpret_cast<const char *>(&msg->scan_time) + sizeof(msg->scan_time));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "scan_time: %f", msg->scan_time);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range_min), reinterpret_cast<const char *>(&msg->range_min) + sizeof(msg->range_min));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "range_min: %f", msg->range_min);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->range_max), reinterpret_cast<const char *>(&msg->range_max) + sizeof(msg->range_max));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "range_max: %f", msg->range_max);

        uint32_t ranges_size = htonl(static_cast<uint32_t>(msg->ranges.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&ranges_size), reinterpret_cast<const char *>(&ranges_size) + sizeof(ranges_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "ranges_size: %u", ranges_size);

        packet.insert(packet.end(), reinterpret_cast<const char *>(msg->ranges.data()), reinterpret_cast<const char *>(msg->ranges.data()) + msg->ranges.size() * sizeof(float));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "first range: %f", msg->ranges[0]);

        uint32_t intensities_size = htonl(static_cast<uint32_t>(msg->intensities.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&intensities_size), reinterpret_cast<const char *>(&intensities_size) + sizeof(intensities_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "intensities_size: %u", intensities_size);

        packet.insert(packet.end(), reinterpret_cast<const char *>(msg->intensities.data()), reinterpret_cast<const char *>(msg->intensities.data()) + msg->intensities.size() * sizeof(float));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::serialize"), "first intensity: %f", msg->intensities[0]);

        return packet;
    }

    sensor_msgs::msg::LaserScan SensorMsgsMsgLaserScan::deserialize(const std::vector<char> &packet)
    {
        sensor_msgs::msg::LaserScan msg;
        size_t offset = 0;

        uint32_t header_frame_id_size;
        memcpy(&header_frame_id_size, packet.data() + offset, sizeof(header_frame_id_size));
        header_frame_id_size = ntohl(header_frame_id_size);
        offset += sizeof(header_frame_id_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "header_frame_id_size: %u", header_frame_id_size);

        msg.header.frame_id = std::string(packet.begin() + offset, packet.begin() + offset + header_frame_id_size);
        offset += header_frame_id_size;

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "header_frame_id: %s", msg.header.frame_id.c_str());

        memcpy(&msg.header.stamp.sec, packet.data() + offset, sizeof(msg.header.stamp.sec));
        offset += sizeof(msg.header.stamp.sec);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "header_stamp_sec: %u", msg.header.stamp.sec);

        memcpy(&msg.header.stamp.nanosec, packet.data() + offset, sizeof(msg.header.stamp.nanosec));
        offset += sizeof(msg.header.stamp.nanosec);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "header_stamp_nanosec: %u", msg.header.stamp.nanosec);

        memcpy(&msg.angle_min, packet.data() + offset, sizeof(msg.angle_min));
        offset += sizeof(msg.angle_min);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "angle_min: %f", msg.angle_min);

        memcpy(&msg.angle_max, packet.data() + offset, sizeof(msg.angle_max));
        offset += sizeof(msg.angle_max);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "angle_max: %f", msg.angle_max);

        memcpy(&msg.angle_increment, packet.data() + offset, sizeof(msg.angle_increment));
        offset += sizeof(msg.angle_increment);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "angle_increment: %f", msg.angle_increment);

        memcpy(&msg.time_increment, packet.data() + offset, sizeof(msg.time_increment));
        offset += sizeof(msg.time_increment);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "time_increment: %f", msg.time_increment);

        memcpy(&msg.scan_time, packet.data() + offset, sizeof(msg.scan_time));
        offset += sizeof(msg.scan_time);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "scan_time: %f", msg.scan_time);

        memcpy(&msg.range_min, packet.data() + offset, sizeof(msg.range_min));
        offset += sizeof(msg.range_min);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "range_min: %f", msg.range_min);

        memcpy(&msg.range_max, packet.data() + offset, sizeof(msg.range_max));
        offset += sizeof(msg.range_max);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "range_max: %f", msg.range_max);

        uint32_t ranges_size;
        memcpy(&ranges_size, packet.data() + offset, sizeof(ranges_size));
        ranges_size = ntohl(ranges_size);
        offset += sizeof(ranges_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "ranges_size: %u", ranges_size);

        msg.ranges.resize(ranges_size);
        memcpy(msg.ranges.data(), packet.data() + offset, ranges_size * sizeof(float));
        offset += ranges_size * sizeof(float);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "first range: %f", msg.ranges[0]);

        uint32_t intensities_size;
        memcpy(&intensities_size, packet.data() + offset, sizeof(intensities_size));
        intensities_size = ntohl(intensities_size);
        offset += sizeof(intensities_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "intensities_size: %u", intensities_size);

        msg.intensities.resize(intensities_size);
        memcpy(msg.intensities.data(), packet.data() + offset, intensities_size * sizeof(float));
        offset += intensities_size * sizeof(float);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_laser_scan::deserialize"), "first intensity: %f", msg.intensities[0]);

        return msg;
    }

} // namespace tcp_ip_bridge