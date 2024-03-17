#include <vector> // std::vector

#include "sensor_msgs/msg/imu.hpp" // sensor_msgs::msg::Imu

namespace tcp_ip_bridge
{

    class SensorMsgsMsgImu
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Imu> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::Imu> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::Imu deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::Imu deserialize(std::vector<char> &packet, sensor_msgs::msg::Imu &msg);
    };

} // namespace tcp_ip_bridge