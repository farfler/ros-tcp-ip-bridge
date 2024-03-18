#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__POINT_CLOUD2_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__POINT_CLOUD2_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/point_cloud2.hpp" // sensor_msgs::msg::PointCloud2

namespace tcp_ip_bridge
{

    class SensorMsgsMsgPointCloud2
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::PointCloud2 deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::PointCloud2 deserialize(std::vector<char> &packet, sensor_msgs::msg::PointCloud2 &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__POINT_CLOUD2_HPP_