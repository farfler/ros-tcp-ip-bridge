#include <vector> // std::vector

#include "sensor_msgs/msg/nav_sat_status.hpp" // sensor_msgs::msg::NavSatStatus

namespace tcp_ip_bridge
{

    class SensorMsgsMsgNavSatStatus
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::NavSatStatus> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::NavSatStatus> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::NavSatStatus deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::NavSatStatus deserialize(std::vector<char> &packet, sensor_msgs::msg::NavSatStatus &msg);
    };

} // namespace tcp_ip_bridge