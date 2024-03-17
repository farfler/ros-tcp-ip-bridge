#include <vector> // std::vector

#include "geometry_msgs/msg/accel_stamped.hpp" // geometry_msgs::msg::AccelStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgAccelStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::AccelStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::AccelStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::AccelStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::AccelStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::AccelStamped &msg);
    };

} // namespace tcp_ip_bridge