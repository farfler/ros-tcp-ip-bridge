#include <vector> // std::vector

#include "sensor_msgs/msg/time_reference.hpp" // sensor_msgs::msg::TimeReference

namespace tcp_ip_bridge
{

    class SensorMsgsMsgTimeReference
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::TimeReference> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::TimeReference> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::TimeReference deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::TimeReference deserialize(std::vector<char> &packet, sensor_msgs::msg::TimeReference &msg);
    };

} // namespace tcp_ip_bridge