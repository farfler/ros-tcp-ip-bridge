#pragma once

#include <memory>

#include "tcp_ip_bridge/sockets/address.hpp"
#include "tcp_ip_bridge/sockets/receiver.hpp"
#include "tcp_ip_bridge/sockets/sender.hpp"
#include "tcp_ip_bridge/sockets/socket.hpp"

namespace tcp_ip_bridge
{

    class Client : public Sender, public Receiver
    {
    public:
        explicit Client(
            const Address &server_address, std::shared_ptr<Socket> socket = make_tcp_socket());

        ~Client();

        size_t send_raw(const char *data, const size_t &length) override;
        size_t receive_raw(char *data, const size_t &length) override;

        std::shared_ptr<Socket> get_socket() const;
        const Address &get_server_address() const;

    protected:
        std::shared_ptr<Socket> socket;
        Address server_address;
    };

} // namespace tcp_ip_bridge
