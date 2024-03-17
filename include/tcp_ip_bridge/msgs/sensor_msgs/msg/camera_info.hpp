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