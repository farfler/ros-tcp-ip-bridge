#pragma once

#include <string>
#include <vector>

namespace tcp_ip_bridge
{

    class Sender
    {
    public:
        virtual size_t send_raw(const char *data, const size_t &length);

        size_t send_string(const std::string &data);
        size_t send_strings(const std::vector<std::string> &data, const std::string &delimiter = ",");

        template <typename T>
        size_t send(const T &data);
    };

} // namespace tcp_ip_bridge

#include "tcp_ip_bridge/sockets/sender.tpp"
