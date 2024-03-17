#include <vector> // std::vector

#include "geometry_msgs/msg/point32.hpp" // geometry_msgs::msg::Point32

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPoint32
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Point32> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Point32> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Point32 deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Point32 deserialize(std::vector<char> &packet, geometry_msgs::msg::Point32 &msg);
    };

} // namespace tcp_ip_bridge