#include <vector> // std::vector

#include "geometry_msgs/msg/pose.hpp" // geometry_msgs::msg::Pose

namespace tcp_ip_bridge
{

    class GeometryMsgsMsgPose
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Pose> &msg);
        static std::vector<char> serialize(const std::shared_ptr<geometry_msgs::msg::Pose> &msg, std::vector<char> &packet);
        static geometry_msgs::msg::Pose deserialize(std::vector<char> &packet);
        static geometry_msgs::msg::Pose deserialize(std::vector<char> &packet, geometry_msgs::msg::Pose &msg);
    };

} // namespace tcp_ip_bridge