#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                // RCLCPP_DEBUG
#include "sensor_msgs/msg/point_cloud2.hpp" // sensor_msgs::msg::PointCloud2

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/point_cloud2.hpp" // SensorMsgsMsgPointCloud2
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"          // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgPointCloud2::serialize(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgPointCloud2::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgPointCloud2::serialize(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->height), reinterpret_cast<const char *>(&msg->height) + sizeof(msg->height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "height: %u", msg->height);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->width), reinterpret_cast<const char *>(&msg->width) + sizeof(msg->width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "width: %u", msg->width);

        uint32_t fields_size = htonl(static_cast<uint32_t>(msg->fields.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&fields_size), reinterpret_cast<const char *>(&fields_size) + sizeof(fields_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "fields_size: %u", ntohl(fields_size));

        if (msg->fields.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->fields.data()), reinterpret_cast<const char *>(msg->fields.data() + msg->fields.size() * sizeof(sensor_msgs::msg::PointField)));
        }

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->is_bigendian), reinterpret_cast<const char *>(&msg->is_bigendian) + sizeof(msg->is_bigendian));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "is_bigendian: %d", msg->is_bigendian);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->point_step), reinterpret_cast<const char *>(&msg->point_step) + sizeof(msg->point_step));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "point_step: %u", msg->point_step);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->row_step), reinterpret_cast<const char *>(&msg->row_step) + sizeof(msg->row_step));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "row_step: %u", msg->row_step);

        uint32_t data_size = htonl(static_cast<uint32_t>(msg->data.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&data_size), reinterpret_cast<const char *>(&data_size) + sizeof(data_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "data_size: %u", ntohl(data_size));

        if (msg->data.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->data.data()), reinterpret_cast<const char *>(msg->data.data() + msg->data.size() * sizeof(char)));
        }

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->is_dense), reinterpret_cast<const char *>(&msg->is_dense) + sizeof(msg->is_dense));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::serialize"), "is_dense: %d", msg->is_dense);

        return packet;
    }

    sensor_msgs::msg::PointCloud2 SensorMsgsMsgPointCloud2::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::PointCloud2 msg;

        SensorMsgsMsgPointCloud2::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::PointCloud2 SensorMsgsMsgPointCloud2::deserialize(std::vector<char> &packet, sensor_msgs::msg::PointCloud2 &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.height, packet.data(), sizeof(msg.height));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "height: %u", msg.height);

        memcpy(&msg.width, packet.data(), sizeof(msg.width));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "width: %u", msg.width);

        uint32_t fields_size;
        memcpy(&fields_size, packet.data(), sizeof(fields_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(fields_size));
        fields_size = ntohl(fields_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "fields_size: %u", fields_size);

        if (fields_size > 0)
        {
            msg.fields.resize(fields_size);
            memcpy(msg.fields.data(), packet.data(), fields_size * sizeof(sensor_msgs::msg::PointField));
            packet.erase(packet.begin(), packet.begin() + fields_size * sizeof(sensor_msgs::msg::PointField));
        }

        memcpy(&msg.is_bigendian, packet.data(), sizeof(msg.is_bigendian));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.is_bigendian));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "is_bigendian: %d", msg.is_bigendian);

        memcpy(&msg.point_step, packet.data(), sizeof(msg.point_step));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.point_step));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "point_step: %u", msg.point_step);

        memcpy(&msg.row_step, packet.data(), sizeof(msg.row_step));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.row_step));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "row_step: %u", msg.row_step);

        uint32_t data_size;
        memcpy(&data_size, packet.data(), sizeof(data_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(data_size));
        data_size = ntohl(data_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "data_size: %u", data_size);

        if (data_size > 0)
        {
            msg.data.resize(data_size);
            memcpy(msg.data.data(), packet.data(), data_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + data_size * sizeof(char));
        }

        memcpy(&msg.is_dense, packet.data(), sizeof(msg.is_dense));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.is_dense));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_point_cloud2::deserialize"), "is_dense: %d", msg.is_dense);

        return msg;
    }

} // namespace tcp_ip_bridge