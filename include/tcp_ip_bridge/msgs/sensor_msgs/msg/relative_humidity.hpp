#include <vector> // std::vector

#include "sensor_msgs/msg/relative_humidity.hpp" // sensor_msgs::msg::RelativeHumidity

namespace tcp_ip_bridge
{

    class SensorMsgsMsgRelativeHumidity
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::RelativeHumidity> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::RelativeHumidity> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::RelativeHumidity deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::RelativeHumidity deserialize(std::vector<char> &packet, sensor_msgs::msg::RelativeHumidity &msg);
    };

} // namespace tcp_ip_bridge