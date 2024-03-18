#include <boost/asio.hpp>
#include <cstring>
#include <vector>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

using namespace tcp_ip_bridge;

template <typename MsgType, typename MsgClass>
class GenericTCPClient : public rclcpp::Node
{
public:
    GenericTCPClient(const std::string &node_name) : Node(node_name)
    {
        this->declare_parameter<std::string>("ip", "0.0.0.0");
        this->get_parameter_or<std::string>("ip", ip_, "0.0.0.0");
        this->declare_parameter<uint16_t>("port", 5000);
        this->get_parameter_or<uint16_t>("port", port_, 5000);
        this->declare_parameter<std::string>("publisher_topic", "");
        this->get_parameter_or<std::string>("publisher_topic", publisher_topic_, "");
        this->declare_parameter<std::string>("subscription_topic", "");
        this->get_parameter_or<std::string>("subscription_topic", subscription_topic_, "");

        auto resolver = boost::asio::ip::tcp::resolver(io_context_);
        auto endpoints = resolver.resolve(ip_, std::to_string(port_));
        boost::asio::connect(socket_, endpoints);

        if (!publisher_topic_.empty())
        {
            publisher_ = this->create_publisher<MsgType>(publisher_topic_, 10);
            receive_thread_ = std::thread(&GenericTCPClient::receive_thread, this);
            RCLCPP_INFO(this->get_logger(), "Forwarding messages from server '%s:%d' to topic '%s'", ip_.c_str(), port_, publisher_topic_.c_str());
        }

        if (!subscription_topic_.empty())
        {
            subscription_ = this->create_subscription<MsgType>(
                subscription_topic_, 10, std::bind(&GenericTCPClient::subscription_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Forwarding messages from topic '%s' to server '%s:%d'", subscription_topic_.c_str(), ip_.c_str(), port_);
        }
    }

    ~GenericTCPClient()
    {
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }
    }

private:
    void subscription_callback(const std::shared_ptr<MsgType> msg)
    {
        try
        {
            auto packet_buffer = MsgClass::serialize(msg);
            uint32_t packet_size = htonl(static_cast<uint32_t>(packet_buffer.size()));

            boost::asio::write(socket_, boost::asio::buffer(&packet_size, sizeof(packet_size)));
            boost::asio::write(socket_, boost::asio::buffer(packet_buffer));
        }
        catch (const boost::system::system_error &e)
        {
            if (e.code() == boost::asio::error::eof ||
                e.code() == boost::asio::error::broken_pipe ||
                e.code() == boost::asio::error::connection_reset)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to send message to the server. Connection was closed.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send message to the server. %s", e.what());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send message to the server. %s", e.what());
        }
    }

    void receive_thread()
    {
        while (rclcpp::ok())
        {
            try
            {
                uint32_t packet_size;
                std::vector<char> packet_size_buffer(sizeof(uint32_t));
                boost::asio::read(socket_, boost::asio::buffer(packet_size_buffer));
                std::memcpy(&packet_size, packet_size_buffer.data(), sizeof(packet_size));
                packet_size = ntohl(packet_size);

                std::vector<char> packet_buffer(packet_size);
                boost::asio::read(socket_, boost::asio::buffer(packet_buffer, packet_size));

                auto msg = MsgClass::deserialize(packet_buffer);
                publisher_->publish(msg);
            }
            catch (const boost::system::system_error &e)
            {
                if (e.code() == boost::asio::error::eof ||
                    e.code() == boost::asio::error::broken_pipe ||
                    e.code() == boost::asio::error::connection_reset)
                {
                    RCLCPP_INFO(this->get_logger(), "Failed to receive message from the server. Connection was closed.");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to receive message from the server. %s", e.what());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive message from the server. %s", e.what());
            }
        }
    }

    std::string ip_;
    uint16_t port_;
    std::string publisher_topic_;
    std::string subscription_topic_;

    boost::asio::io_context io_context_ = boost::asio::io_context();
    boost::asio::ip::tcp::socket socket_ = boost::asio::ip::tcp::socket(io_context_);

    std::shared_ptr<rclcpp::Subscription<MsgType>> subscription_;
    std::shared_ptr<rclcpp::Publisher<MsgType>> publisher_;

    std::thread receive_thread_;
};
