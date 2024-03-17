#include <vector> // std::vector

#include "geometry_msgs/msg/pose_stamped.hpp" // geometry_msgs::msg::PoseStamped

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPoseStamped
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::PoseStamped deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::PoseStamped deserialize(std::vector<char> &packet, geometry_msgs::msg::PoseStamped &msg);
    };

} // namespace tcp_ip_bridge