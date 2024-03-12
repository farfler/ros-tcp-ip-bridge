#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tcp_ip_bridge/sockets/tcp/client.hpp"
#include "tcp_ip_bridge/msgs/sensor_msgs/msg/laser_scan.hpp"

class SensorMsgsMsgLaserScanTCPClient : public rclcpp::Node
{
public:
    SensorMsgsMsgLaserScanTCPClient() : Node("sensor_msgs_msg_laser_scan_tcp_client")
    {
        this->declare_parameter<std::string>("ip", "0.0.0.0");
        this->get_parameter_or<std::string>("ip", ip_, "0.0.0.0");
        this->declare_parameter<uint16_t>("port", 5000);
        this->get_parameter_or<uint16_t>("port", port_, 5000);
        this->declare_parameter<std::string>("publisher_topic", "");
        this->get_parameter_or<std::string>("publisher_topic", publisher_topic_, "");
        this->declare_parameter<std::string>("subscription_topic", "");
        this->get_parameter_or<std::string>("subscription_topic", subscription_topic_, "");

        client_ = std::make_unique<tcp_ip_bridge::Client>(tcp_ip_bridge::Address(ip_, port_));

        if (!publisher_topic_.empty())
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(publisher_topic_, 10);

            receive_thread_ = std::thread(&SensorMsgsMsgLaserScanTCPClient::receive_thread, this);

            RCLCPP_INFO(this->get_logger(), "Receiving messages from server %s:%d to topic '%s'", ip_.c_str(), port_, publisher_topic_.c_str());
        }

        if (!subscription_topic_.empty())
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                subscription_topic_, 10, std::bind(&SensorMsgsMsgLaserScanTCPClient::subscription_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Forwarding messages from topic '%s' to server %s:%d", subscription_topic_.c_str(), ip_.c_str(), port_);
        }
    }

    ~SensorMsgsMsgLaserScanTCPClient()
    {
        receive_thread_.join();
    }

private:
    void subscription_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try
        {
            tcp_ip_bridge::SensorMsgsMsgLaserScan packet;

            packet.header_frame_id = msg->header.frame_id;
            packet.header_stamp_sec = msg->header.stamp.sec;
            packet.header_stamp_nanosec = msg->header.stamp.nanosec;
            packet.angle_min = msg->angle_min;
            packet.angle_max = msg->angle_max;
            packet.angle_increment = msg->angle_increment;
            packet.time_increment = msg->time_increment;
            packet.scan_time = msg->scan_time;
            packet.range_min = msg->range_min;
            packet.range_max = msg->range_max;
            packet.ranges = msg->ranges;
            packet.intensities = msg->intensities;

            std::vector<char> vector;
            tcp_ip_bridge::serialize_sensor_msgs_msg_laser_scan(packet, vector);

            size_t packet_size = vector.size();
            uint32_t packet_size_net = htonl(packet_size);

            vector.insert(vector.begin(), reinterpret_cast<char *>(&packet_size_net), reinterpret_cast<char *>(&packet_size_net) + sizeof(packet_size_net));
            client_->send_raw(vector.data(), vector.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send sensor_msgs_msg_laser_scan message to the server. %s", e.what());
        }
    }

    void receive_thread()
    {
        while (rclcpp::ok())
        {
            try
            {
                auto packet_size_net = client_->receive<uint32_t>();

                if (!packet_size_net.has_value())
                {
                    continue;
                }

                size_t packet_size = ntohl(packet_size_net.value());

                auto buffer = std::make_unique<char[]>(packet_size);
                client_->receive_raw(buffer.get(), packet_size);
                std::vector<char> vector(buffer.get(), buffer.get() + packet_size);

                tcp_ip_bridge::SensorMsgsMsgLaserScan packet;
                tcp_ip_bridge::deserialize_sensor_msgs_msg_laser_scan(vector, packet);
                sensor_msgs::msg::LaserScan msg;

                msg.header.frame_id = packet.header_frame_id;
                msg.header.stamp.sec = packet.header_stamp_sec;
                msg.header.stamp.nanosec = packet.header_stamp_nanosec;
                msg.angle_min = packet.angle_min;
                msg.angle_max = packet.angle_max;
                msg.angle_increment = packet.angle_increment;
                msg.time_increment = packet.time_increment;
                msg.scan_time = packet.scan_time;
                msg.range_min = packet.range_min;
                msg.range_max = packet.range_max;
                msg.ranges = packet.ranges;
                msg.intensities = packet.intensities;

                publisher_->publish(msg);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive sensor_msgs_msg_laser_scan message from the server. %s", e.what());
            }
        }
    }

    std::string ip_;
    uint16_t port_;
    std::string publisher_topic_;
    std::string subscription_topic_;

    std::unique_ptr<tcp_ip_bridge::Client> client_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    std::thread receive_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMsgsMsgLaserScanTCPClient>());
    rclcpp::shutdown();
    return 0;
}