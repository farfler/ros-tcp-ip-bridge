#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__REGION_OF_INTEREST_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__REGION_OF_INTEREST_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/region_of_interest.hpp" // sensor_msgs::msg::RegionOfInterest

namespace tcp_ip_bridge
{

    class SensorMsgsMsgRegionOfInterest
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::RegionOfInterest> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::RegionOfInterest> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::RegionOfInterest deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::RegionOfInterest deserialize(std::vector<char> &packet, sensor_msgs::msg::RegionOfInterest &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__REGION_OF_INTEREST_HPP_