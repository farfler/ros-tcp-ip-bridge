#pragma once

#include <memory>
#include <string>

#include "tcp_ip_bridge/sockets/tcp/session.hpp"
#include "tcp_ip_bridge/sockets/socket.hpp"

namespace tcp_ip_bridge
{

    class Server
    {
    public:
        explicit Server(const int &port, std::shared_ptr<Socket> socket = make_tcp_socket());
        ~Server();

        std::shared_ptr<Session> accept();

        std::shared_ptr<Socket> get_socket() const;
        const int &get_port() const;

    protected:
        std::shared_ptr<Socket> socket;
        int port;
    };

} // namespace tcp_ip_bridge
