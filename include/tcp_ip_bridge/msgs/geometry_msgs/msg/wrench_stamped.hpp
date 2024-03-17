#include <vector> // std::vector

#include "geometry_msgs/msg/wrench_stamped.hpp" // geometry_msgs::msg::WrenchStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgWrenchStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::WrenchStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::WrenchStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::WrenchStamped &msg);
    };

} // namespace tcp_ip_bridge