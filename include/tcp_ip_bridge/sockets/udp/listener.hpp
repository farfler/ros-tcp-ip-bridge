#pragma once

#include <memory>

#include "tcp_ip_bridge/sockets/receiver.hpp"
#include "tcp_ip_bridge/sockets/socket.hpp"

namespace tcp_ip_bridge
{

    class Listener : public Receiver
    {
    public:
        explicit Listener(const int &port, std::shared_ptr<Socket> socket = make_udp_socket());
        ~Listener();

        size_t receive_raw(char *data, const size_t &length) override;

        std::shared_ptr<Socket> get_socket() const;
        const int &get_port() const;

    protected:
        std::shared_ptr<Socket> socket;
        int port;
    };

} // namespace tcp_ip_bridge
