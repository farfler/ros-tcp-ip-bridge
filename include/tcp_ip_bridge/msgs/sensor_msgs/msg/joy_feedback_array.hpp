#include <vector> // std::vector

#include "sensor_msgs/msg/joy_feedback_array.hpp" // sensor_msgs::msg::JoyFeedbackArray

namespace tcp_ip_bridge
{

    class SensorMsgsMsgJoyFeedbackArray
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::JoyFeedbackArray deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::JoyFeedbackArray deserialize(std::vector<char> &packet, sensor_msgs::msg::JoyFeedbackArray &msg);
    };

} // namespace tcp_ip_bridge