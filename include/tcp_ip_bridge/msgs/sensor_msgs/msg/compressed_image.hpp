#include <vector> // std::vector

#include "sensor_msgs/msg/compressed_image.hpp" // sensor_msgs::msg::CompressedImage

namespace tcp_ip_bridge
{

    class SensorMsgsMsgCompressedImage
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::CompressedImage> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::CompressedImage> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::CompressedImage deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::CompressedImage deserialize(std::vector<char> &packet, sensor_msgs::msg::CompressedImage &msg);
    };

} // namespace tcp_ip_bridge