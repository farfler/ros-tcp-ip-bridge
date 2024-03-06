#pragma once

#include "tcp_ip_bridge/sockets/sender.hpp"

namespace tcp_ip_bridge
{

    template <typename T>
    size_t Sender::send(const T &data)
    {
        return send_raw((const char *)&data, sizeof(data));
    }

} // namespace tcp_ip_bridge
