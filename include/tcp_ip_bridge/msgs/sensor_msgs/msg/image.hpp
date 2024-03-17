#include <vector> // std::vector

#include "sensor_msgs/msg/image.hpp" // sensor_msgs::msg::Image

namespace tcp_ip_bridge
{

    class SensorMsgsMsgImage
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Image> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Image> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::Image deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::Image deserialize(std::vector<char> &packet, sensor_msgs::msg::Image &msg);
    };

} // namespace tcp_ip_bridge