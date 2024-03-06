#pragma once

#include <sys/socket.h>
#include <system_error>

#include "tcp_ip_bridge/sockets/socket.hpp"

namespace tcp_ip_bridge
{

    template <typename T>
    void Socket::set_option(const int &key, const T &value)
    {
        if (setsockopt(fd, SOL_SOCKET, key, &value, sizeof(value)) == -1)
        {
            throw std::system_error(errno, std::generic_category());
        }
    }

    template <typename T>
    T Socket::get_option(const int &key) const
    {
        T value;
        socklen_t value_len = sizeof(value);

        if (getsockopt(fd, SOL_SOCKET, key, &value, &value_len) == -1)
        {
            throw std::system_error(errno, std::generic_category());
        }

        return value;
    }

} // namespace tcp_ip_bridge
