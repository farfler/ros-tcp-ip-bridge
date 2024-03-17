#include <vector> // std::vector

#include "geometry_msgs/msg/transform.hpp" // geometry_msgs::msg::Transform

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgTransform
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Transform> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Transform> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Transform deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Transform deserialize(std::vector<char> &packet, geometry_msgs::msg::Transform &msg);
    };

} // namespace tcp_ip_bridge