#include <fcntl.h>

#include <memory>

#include "tcp_ip_bridge/sockets/tcp/server.hpp"

namespace tcp_ip_bridge
{

    Server::Server(const int &port, std::shared_ptr<Socket> socket)
    {
        this->socket = socket;
        this->port = port;

        // Bind the socket with the serve address
        socket->bind(make_any_address(port));

        // Listen to incoming connections
        socket->listen();
    }

    Server::~Server()
    {
        socket = nullptr;
    }

    std::shared_ptr<Session> Server::accept()
    {
        try
        {
            // Accept incoming connection
            auto session_socket = socket->accept();

            // Set the session socket's non-blocking status according to the server socket's
            auto was_nonblock = socket->get_status_flag(O_NONBLOCK);
            session_socket->set_status_flag(O_NONBLOCK, was_nonblock);

            return std::make_shared<Session>(session_socket);
        }
        catch (const std::system_error &err)
        {
            if (err.code().value() == EAGAIN)
            {
                return nullptr;
            }

            throw err;
        }
    }

    std::shared_ptr<Socket> Server::get_socket() const
    {
        return socket;
    }

    const int &Server::get_port() const
    {
        return port;
    }

} // namespace tcp_ip_bridge
