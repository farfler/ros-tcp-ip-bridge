#include <vector> // std::vector

#include "geometry_msgs/msg/inertia_stamped.hpp" // geometry_msgs::msg::InertiaStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgInertiaStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::InertiaStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::InertiaStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::InertiaStamped &msg);
    };

} // namespace tcp_ip_bridge