#include <vector> // std::vector

#include "sensor_msgs/msg/nav_sat_fix.hpp" // sensor_msgs::msg::NavSatFix

namespace tcp_ip_bridge
{

    class SensorMsgsMsgNavSatFix
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::NavSatFix deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::NavSatFix deserialize(std::vector<char> &packet, sensor_msgs::msg::NavSatFix &msg);
    };

} // namespace tcp_ip_bridge