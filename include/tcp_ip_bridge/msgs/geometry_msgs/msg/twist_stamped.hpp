#include <vector> // std::vector

#include "geometry_msgs/msg/twist_stamped.hpp" // geometry_msgs::msg::TwistStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgTwistStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TwistStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TwistStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::TwistStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::TwistStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::TwistStamped &msg);
    };

} // namespace tcp_ip_bridge