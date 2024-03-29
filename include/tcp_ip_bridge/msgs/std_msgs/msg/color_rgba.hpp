#ifndef TCP_IP_BRIDGE__MSGS__STD_MSGS__MSG__COLOR_RGBA_HPP_
#define TCP_IP_BRIDGE__MSGS__STD_MSGS__MSG__COLOR_RGBA_HPP_

#include <vector> // std::vector

#include "std_msgs/msg/color_rgba.hpp" // std_msgs::msg::ColorRGBA

namespace tcp_ip_bridge
{

    class StdMsgsMsgColorRGBA
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<std_msgs::msg::ColorRGBA> &msg);
        static std::vector<char> serialize(const std::shared_ptr<std_msgs::msg::ColorRGBA> &msg, std::vector<char> &packet);
        static std_msgs::msg::ColorRGBA deserialize(std::vector<char> &packet);
        static std_msgs::msg::ColorRGBA deserialize(std::vector<char> &packet, std_msgs::msg::ColorRGBA &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__STD_MSGS__MSG__COLOR_RGBA_HPP_