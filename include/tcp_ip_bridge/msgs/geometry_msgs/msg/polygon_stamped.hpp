#ifndef TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POLYGON_STAMPED_HPP_
#define TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POLYGON_STAMPED_HPP_

#include <vector> // std::vector

#include "geometry_msgs/msg/polygon_stamped.hpp" // geometry_msgs::msg::PolygonStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPolygonStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PolygonStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PolygonStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::PolygonStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::PolygonStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::PolygonStamped &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__GEOMETRY_MSGS__MSG__POLYGON_STAMPED_HPP_