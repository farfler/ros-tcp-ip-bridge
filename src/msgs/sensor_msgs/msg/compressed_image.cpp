#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                    // RCLCPP_DEBUG
#include "sensor_msgs/msg/compressed_image.hpp" // sensor_msgs::msg::CompressedImage

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/compressed_image.hpp" // SensorMsgsMsgCompressedImage
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"              // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgCompressedImage::serialize(const std::shared_ptr<sensor_msgs::msg::CompressedImage> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgCompressedImage::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgCompressedImage::serialize(const std::shared_ptr<sensor_msgs::msg::CompressedImage> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        uint32_t format_size = htonl(static_cast<uint32_t>(msg->format.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&format_size), reinterpret_cast<const char *>(&format_size) + sizeof(format_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_compressed_image::serialize"), "format_size: %u", ntohl(format_size));

        if (msg->format.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->format.data()), reinterpret_cast<const char *>(msg->format.data() + msg->format.size()));

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_compressed_image::serialize"), "format: %s", msg->format.c_str());
        }

        uint32_t data_size = htonl(static_cast<uint32_t>(msg->data.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&data_size), reinterpret_cast<const char *>(&data_size) + sizeof(data_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_compressed_image::serialize"), "data_size: %u", ntohl(data_size));

        if (msg->data.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->data.data()), reinterpret_cast<const char *>(msg->data.data() + msg->data.size()));
        }

        return packet;
    }

    sensor_msgs::msg::CompressedImage SensorMsgsMsgCompressedImage::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::CompressedImage msg;

        SensorMsgsMsgCompressedImage::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::CompressedImage SensorMsgsMsgCompressedImage::deserialize(std::vector<char> &packet, sensor_msgs::msg::CompressedImage &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        uint32_t format_size;
        memcpy(&format_size, packet.data(), sizeof(format_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(format_size));
        format_size = ntohl(format_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_compressed_image::deserialize"), "format_size: %u", format_size);

        if (format_size > 0)
        {
            msg.format.resize(format_size);
            memcpy(msg.format.data(), packet.data(), format_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + format_size * sizeof(char));

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_compressed_image::deserialize"), "format: %s", msg.format.c_str());
        }

        uint32_t data_size;
        memcpy(&data_size, packet.data(), sizeof(data_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(data_size));
        data_size = ntohl(data_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_compressed_image::deserialize"), "data_size: %u", data_size);

        if (data_size > 0)
        {
            msg.data.resize(data_size);
            memcpy(msg.data.data(), packet.data(), data_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + data_size * sizeof(char));
        }

        return msg;
    }

} // namespace tcp_ip_bridge