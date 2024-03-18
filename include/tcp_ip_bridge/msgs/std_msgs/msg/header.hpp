#ifndef TCP_IP_BRIDGE__MSGS__STD_MSGS__MSG__HEADER_HPP_
#define TCP_IP_BRIDGE__MSGS__STD_MSGS__MSG__HEADER_HPP_

#include <vector> // std::vector

#include "std_msgs/msg/header.hpp" // std_msgs::msg::Header

namespace tcp_ip_bridge
{

    class StdMsgsMsgHeader
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<std_msgs::msg::Header> &msg);
        static std::vector<char> serialize(const std::shared_ptr<std_msgs::msg::Header> &msg, std::vector<char> &packet);
        static std_msgs::msg::Header deserialize(std::vector<char> &packet);
        static std_msgs::msg::Header deserialize(std::vector<char> &packet, std_msgs::msg::Header &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__STD_MSGS__MSG__HEADER_HPP_