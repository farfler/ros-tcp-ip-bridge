#ifndef TCP_IP_BRIDGE__MSGS__BUILTIN_INTERFACES__MSG__TIME_HPP_
#define TCP_IP_BRIDGE__MSGS__BUILTIN_INTERFACES__MSG__TIME_HPP_

#include <vector> // std::vector

#include "builtin_interfaces/msg/time.hpp" // builtin_interfaces::msg::Time

namespace tcp_ip_bridge
{

    class BuiltinInterfacesMsgTime
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<builtin_interfaces::msg::Time> &msg);
        static std::vector<char> serialize(const std::shared_ptr<builtin_interfaces::msg::Time> &msg, std::vector<char> &packet);
        static builtin_interfaces::msg::Time deserialize(std::vector<char> &packet);
        static builtin_interfaces::msg::Time deserialize(std::vector<char> &packet, builtin_interfaces::msg::Time &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__BUILTIN_INTERFACES__MSG__TIME_HPP_