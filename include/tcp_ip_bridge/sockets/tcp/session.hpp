#pragma once

#include <memory>

#include "tcp_ip_bridge/sockets/sender.hpp"
#include "tcp_ip_bridge/sockets/receiver.hpp"
#include "tcp_ip_bridge/sockets/socket.hpp"

namespace tcp_ip_bridge
{

    class Session : public Sender, public Receiver
    {
    public:
        explicit Session(std::shared_ptr<Socket> socket);
        ~Session();

        size_t send_raw(const char *data, const size_t &length) override;
        size_t receive_raw(char *data, const size_t &length) override;

        std::shared_ptr<Socket> get_socket() const;

    protected:
        std::shared_ptr<Socket> socket;
    };

} // namespace tcp_ip_bridge
