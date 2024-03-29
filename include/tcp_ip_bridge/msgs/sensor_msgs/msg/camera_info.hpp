#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__CAMERA_INFO_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__CAMERA_INFO_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/camera_info.hpp" // sensor_msgs::msg::CameraInfo

namespace tcp_ip_bridge
{

    class SensorMsgsMsgCameraInfo
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::CameraInfo> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::CameraInfo> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::CameraInfo deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::CameraInfo deserialize(std::vector<char> &packet, sensor_msgs::msg::CameraInfo &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__CAMERA_INFO_HPP_