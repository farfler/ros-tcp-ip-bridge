#include <vector> // std::vector

#include "sensor_msgs/msg/multi_dof_joint_state.hpp" // sensor_msgs::msg::MultiDOFJointState

namespace tcp_ip_bridge
{

    class SensorMsgsMsgMultiDOFJointState
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::MultiDOFJointState> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::MultiDOFJointState> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::MultiDOFJointState deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::MultiDOFJointState deserialize(std::vector<char> &packet, sensor_msgs::msg::MultiDOFJointState &msg);
    };

} // namespace tcp_ip_bridge