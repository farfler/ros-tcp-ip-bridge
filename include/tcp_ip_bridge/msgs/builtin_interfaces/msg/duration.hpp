#ifndef TCP_IP_BRIDGE__MSGS__BUILTIN_INTERFACES__MSG__DURATION_HPP_
#define TCP_IP_BRIDGE__MSGS__BUILTIN_INTERFACES__MSG__DURATION_HPP_

#include <vector> // std::vector

#include "builtin_interfaces/msg/duration.hpp" // builtin_interfaces::msg::Duration

namespace tcp_ip_bridge
{

    class BuiltinInterfacesMsgDuration
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<builtin_interfaces::msg::Duration> &msg);
        static std::vector<char> serialize(const std::shared_ptr<builtin_interfaces::msg::Duration> &msg, std::vector<char> &packet);
        static builtin_interfaces::msg::Duration deserialize(std::vector<char> &packet);
        static builtin_interfaces::msg::Duration deserialize(std::vector<char> &packet, builtin_interfaces::msg::Duration &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__BUILTIN_INTERFACES__MSG__DURATION_HPP_