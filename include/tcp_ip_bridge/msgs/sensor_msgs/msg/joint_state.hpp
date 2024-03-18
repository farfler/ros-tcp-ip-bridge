#include <vector> // std::vector

#include "sensor_msgs/msg/joint_state.hpp" // sensor_msgs::msg::JointState

namespace tcp_ip_bridge
{

    class SensorMsgsMsgJointState
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::JointState> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::JointState> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::JointState deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::JointState deserialize(std::vector<char> &packet, sensor_msgs::msg::JointState &msg);
    };

} // namespace tcp_ip_bridge