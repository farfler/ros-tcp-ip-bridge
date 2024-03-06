#pragma once

#include <list>
#include <memory>
#include <string>

#include "tcp_ip_bridge/sockets/address.hpp"
#include "tcp_ip_bridge/sockets/sender.hpp"
#include "tcp_ip_bridge/sockets/socket.hpp"

namespace tcp_ip_bridge
{

    class Broadcaster : public Sender
    {
    public:
        explicit Broadcaster(const int &port, std::shared_ptr<Socket> socket = make_udp_socket());
        ~Broadcaster();

        size_t send_raw(const char *data, const size_t &length) override;

        void enable_broadcast(const bool &enable);

        std::shared_ptr<Socket> get_socket() const;
        const int &get_port() const;

        std::list<std::string> target_ips;

    protected:
        std::shared_ptr<Socket> socket;
        int port;

        std::list<Address> broadcast_addresses;
    };

} // namespace tcp_ip_bridge
