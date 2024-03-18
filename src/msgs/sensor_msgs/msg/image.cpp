#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"         // RCLCPP_DEBUG
#include "sensor_msgs/msg/image.hpp" // sensor_msgs::msg::Image

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/image.hpp" // SensorMsgsMsgImage
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"   // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgImage::serialize(const std::shared_ptr<sensor_msgs::msg::Image> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgImage::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgImage::serialize(const std::shared_ptr<sensor_msgs::msg::Image> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->height), reinterpret_cast<const char *>(&msg->height) + sizeof(msg->height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "height: %d", msg->height);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->width), reinterpret_cast<const char *>(&msg->width) + sizeof(msg->width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "width: %d", msg->width);

        uint32_t encoding_size = htonl(static_cast<uint32_t>(msg->encoding.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&encoding_size), reinterpret_cast<const char *>(&encoding_size) + sizeof(encoding_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "encoding_size: %u", ntohl(encoding_size));

        if (msg->encoding.size() > 0)
        {
            packet.insert(packet.end(), msg->encoding.begin(), msg->encoding.end());

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "encoding: %s", msg->encoding.c_str());
        }

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->is_bigendian), reinterpret_cast<const char *>(&msg->is_bigendian) + sizeof(msg->is_bigendian));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "is_bigendian: %d", msg->is_bigendian);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->step), reinterpret_cast<const char *>(&msg->step) + sizeof(msg->step));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "step: %d", msg->step);

        uint32_t data_size = htonl(static_cast<uint32_t>(msg->data.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&data_size), reinterpret_cast<const char *>(&data_size) + sizeof(data_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::serialize"), "data_size: %u", ntohl(data_size));

        if (msg->data.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->data.data()), reinterpret_cast<const char *>(msg->data.data() + msg->data.size() * sizeof(char)));
        }

        return packet;
    }

    sensor_msgs::msg::Image SensorMsgsMsgImage::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::Image msg;

        SensorMsgsMsgImage::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::Image SensorMsgsMsgImage::deserialize(std::vector<char> &packet, sensor_msgs::msg::Image &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.height, packet.data(), sizeof(msg.height));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "height: %d", msg.height);

        memcpy(&msg.width, packet.data(), sizeof(msg.width));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "width: %d", msg.width);

        uint32_t encoding_size;
        memcpy(&encoding_size, packet.data(), sizeof(encoding_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(encoding_size));
        encoding_size = ntohl(encoding_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "encoding_size: %u", encoding_size);

        if (encoding_size > 0)
        {
            msg.encoding.resize(encoding_size);
            memcpy(msg.encoding.data(), packet.data(), encoding_size);
            packet.erase(packet.begin(), packet.begin() + encoding_size);

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "encoding: %s", msg.encoding.c_str());
        }

        memcpy(&msg.is_bigendian, packet.data(), sizeof(msg.is_bigendian));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.is_bigendian));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "is_bigendian: %d", msg.is_bigendian);

        memcpy(&msg.step, packet.data(), sizeof(msg.step));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.step));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "step: %d", msg.step);

        uint32_t data_size;
        memcpy(&data_size, packet.data(), sizeof(data_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(data_size));
        data_size = ntohl(data_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_image::deserialize"), "data_size: %u", data_size);

        if (data_size > 0)
        {
            msg.data.resize(data_size);
            memcpy(msg.data.data(), packet.data(), data_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + data_size * sizeof(char));
        }

        return msg;
    }

} // namespace tcp_ip_bridge