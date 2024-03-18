#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POLYGON_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POLYGON_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/polygon.hpp" // geometry_msgs::msg::Polygon

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPolygon
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Polygon> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Polygon> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Polygon deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Polygon deserialize(std::vector<char> &packet, geometry_msgs::msg::Polygon &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POLYGON_HPP_