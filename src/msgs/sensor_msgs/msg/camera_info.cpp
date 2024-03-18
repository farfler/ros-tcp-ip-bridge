#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"               // RCLCPP_DEBUG
#include "sensor_msgs/msg/camera_info.hpp" // sensor_msgs::msg::CameraInfo

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/camera_info.hpp"        // SensorMsgsMsgCameraInfo
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"                // StdMsgsMsgHeader
#include "tcp_ip_bridge/msgs/sensor_msgs/msg/region_of_interest.hpp" // SensorMsgsMsgRegionOfInterest

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgCameraInfo::serialize(const std::shared_ptr<sensor_msgs::msg::CameraInfo> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgCameraInfo::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgCameraInfo::serialize(const std::shared_ptr<sensor_msgs::msg::CameraInfo> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->height), reinterpret_cast<const char *>(&msg->height) + sizeof(msg->height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "height: %d", msg->height);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->width), reinterpret_cast<const char *>(&msg->width) + sizeof(msg->width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "width: %d", msg->width);

        uint32_t distortion_model_size = htonl(static_cast<uint32_t>(msg->distortion_model.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&distortion_model_size), reinterpret_cast<const char *>(&distortion_model_size) + sizeof(distortion_model_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "distortion_model_size: %u", ntohl(distortion_model_size));

        if (msg->distortion_model.size() > 0)
        {
            packet.insert(packet.end(), msg->distortion_model.begin(), msg->distortion_model.end());

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "distortion_model: %s", msg->distortion_model.c_str());
        }

        uint32_t d_size = htonl(static_cast<uint32_t>(msg->d.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&d_size), reinterpret_cast<const char *>(&d_size) + sizeof(d_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "d_size: %u", ntohl(d_size));

        if (msg->d.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->d.data()), reinterpret_cast<const char *>(msg->d.data() + msg->d.size() * sizeof(double)));
        }

        packet.insert(packet.end(), reinterpret_cast<const char *>(msg->k.data()), reinterpret_cast<const char *>(msg->k.data() + msg->k.size()));

        packet.insert(packet.end(), reinterpret_cast<const char *>(msg->r.data()), reinterpret_cast<const char *>(msg->r.data() + msg->r.size()));

        packet.insert(packet.end(), reinterpret_cast<const char *>(msg->p.data()), reinterpret_cast<const char *>(msg->p.data() + msg->p.size()));

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->binning_x), reinterpret_cast<const char *>(&msg->binning_x) + sizeof(msg->binning_x));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "binning_x: %d", msg->binning_x);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->binning_y), reinterpret_cast<const char *>(&msg->binning_y) + sizeof(msg->binning_y));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::serialize"), "binning_y: %d", msg->binning_y);

        SensorMsgsMsgRegionOfInterest::serialize(std::make_shared<sensor_msgs::msg::RegionOfInterest>(msg->roi), packet);

        return packet;
    }

    sensor_msgs::msg::CameraInfo SensorMsgsMsgCameraInfo::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::CameraInfo msg;

        SensorMsgsMsgCameraInfo::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::CameraInfo SensorMsgsMsgCameraInfo::deserialize(std::vector<char> &packet, sensor_msgs::msg::CameraInfo &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.height, packet.data(), sizeof(msg.height));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.height));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::deserialize"), "height: %d", msg.height);

        memcpy(&msg.width, packet.data(), sizeof(msg.width));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.width));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::deserialize"), "width: %d", msg.width);

        uint32_t distortion_model_size;
        memcpy(&distortion_model_size, packet.data(), sizeof(distortion_model_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(distortion_model_size));
        distortion_model_size = ntohl(distortion_model_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::deserialize"), "distortion_model_size: %u", distortion_model_size);

        if (distortion_model_size > 0)
        {
            msg.distortion_model.resize(distortion_model_size);
            memcpy(msg.distortion_model.data(), packet.data(), distortion_model_size);
            packet.erase(packet.begin(), packet.begin() + distortion_model_size);
        }

        uint32_t d_size;
        memcpy(&d_size, packet.data(), sizeof(d_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(d_size));
        d_size = ntohl(d_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::deserialize"), "d_size: %u", d_size);

        if (d_size > 0)
        {
            msg.d.resize(d_size);
            memcpy(msg.d.data(), packet.data(), d_size * sizeof(double));
            packet.erase(packet.begin(), packet.begin() + d_size * sizeof(double));
        }

        memcpy(msg.k.data(), packet.data(), msg.k.size() * sizeof(double));
        packet.erase(packet.begin(), packet.begin() + msg.k.size() * sizeof(double));

        memcpy(msg.r.data(), packet.data(), msg.r.size() * sizeof(double));
        packet.erase(packet.begin(), packet.begin() + msg.r.size() * sizeof(double));

        memcpy(msg.p.data(), packet.data(), msg.p.size() * sizeof(double));
        packet.erase(packet.begin(), packet.begin() + msg.p.size() * sizeof(double));

        memcpy(&msg.binning_x, packet.data(), sizeof(msg.binning_x));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.binning_x));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::deserialize"), "binning_x: %d", msg.binning_x);

        memcpy(&msg.binning_y, packet.data(), sizeof(msg.binning_y));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.binning_y));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_camera_info::deserialize"), "binning_y: %d", msg.binning_y);

        SensorMsgsMsgRegionOfInterest::deserialize(packet, msg.roi);

        return msg;
    }

} // namespace tcp_ip_bridge