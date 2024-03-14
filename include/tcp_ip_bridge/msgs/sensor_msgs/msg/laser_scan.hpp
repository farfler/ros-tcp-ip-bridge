#include <vector>
#include <iostream>
#include <cstring>

namespace tcp_ip_bridge
{

    class SensorMsgsMsgLaserScan
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::LaserScan> &msg);
        static std::shared_ptr<sensor_msgs::msg::LaserScan> deserialize(const std::vector<char> &packet);
    };

} // namespace tcp_ip_bridge