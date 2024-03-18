#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POINT_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POINT_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/point.hpp" // geometry_msgs::msg::Point

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPoint
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Point> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Point> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Point deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Point deserialize(std::vector<char> &packet, geometry_msgs::msg::Point &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POINT_HPP_