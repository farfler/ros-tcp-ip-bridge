#include <vector> // std::vector

#include "geometry_msgs/msg/twist_with_covariance.hpp" // geometry_msgs::msg::TwistWithCovariance

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgTwistWithCovariance
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::TwistWithCovariance deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::TwistWithCovariance deserialize(std::vector<char> &packet, geometry_msgs::msg::TwistWithCovariance &msg);
    };

} // namespace tcp_ip_bridge