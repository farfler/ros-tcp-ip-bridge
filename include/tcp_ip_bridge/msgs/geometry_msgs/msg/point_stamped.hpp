#include <vector> // std::vector

#include "geometry_msgs/msg/point_stamped.hpp" // geometry_msgs::msg::PointStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPointStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PointStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PointStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::PointStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::PointStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::PointStamped &msg);
    };

} // namespace tcp_ip_bridge