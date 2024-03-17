#include <vector> // std::vector

#include "geometry_msgs/msg/vector3_stamped.hpp" // geometry_msgs::msg::Vector3Stamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgVector3Stamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Vector3Stamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Vector3Stamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Vector3Stamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Vector3Stamped deserialize(std::vector<char> &packet, geometry_msgs::msg::Vector3Stamped &msg);
    };

} // namespace tcp_ip_bridge