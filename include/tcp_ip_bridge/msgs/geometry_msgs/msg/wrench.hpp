#include <vector> // std::vector

#include "geometry_msgs/msg/wrench.hpp" // geometry_msgs::msg::Wrench

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgWrench
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Wrench> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Wrench> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Wrench deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Wrench deserialize(std::vector<char> &packet, geometry_msgs::msg::Wrench &msg);
    };

} // namespace tcp_ip_bridge