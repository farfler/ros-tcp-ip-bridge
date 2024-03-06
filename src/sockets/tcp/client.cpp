#include <fcntl.h>

#include <memory>

#include "tcp_ip_bridge/sockets/tcp/client.hpp"

namespace tcp_ip_bridge
{

    Client::Client(const Address &server_address, std::shared_ptr<Socket> socket)
    {
        this->socket = socket;
        this->server_address = server_address;

        // Disable non blocking during connect
        auto was_non_blocking = socket->get_status_flag(O_NONBLOCK);
        if (was_non_blocking)
        {
            socket->set_status_flag(O_NONBLOCK, false);
        }

        // Connect client to the server
        socket->connect(server_address);

        if (was_non_blocking)
        {
            socket->set_status_flag(O_NONBLOCK, true);
        }
    }

    Client::~Client()
    {
        socket = nullptr;
    }

    size_t Client::send_raw(const char *data, const size_t &length)
    {
        return socket->send(data, length);
    }

    size_t Client::receive_raw(char *data, const size_t &length)
    {
        return socket->receive(data, length);
    }

    std::shared_ptr<Socket> Client::get_socket() const
    {
        return socket;
    }

    const Address &Client::get_server_address() const
    {
        return server_address;
    }

} // namespace tcp_ip_bridge
