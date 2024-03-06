#pragma once

#include <utility>

#include "tcp_ip_bridge/sockets/receiver.hpp"

namespace tcp_ip_bridge
{

    template <typename T>
    std::optional<T> Receiver::receive()
    {
        T data;

        auto received = receive_raw(reinterpret_cast<char *>(&data), sizeof(T));
        if (received < sizeof(T))
        {
            return std::nullopt;
        }

        return std::make_optional<T>(std::move(data));
    }

} // namespace tcp_ip_bridge
