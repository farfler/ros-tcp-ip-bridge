#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
    }

private:
    std::string ip_;
    uint16_t port_;

    std::string publisher_topic_;
    std::string subscription_topic_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMsgsMsgLaserScanTCPClient>());
    rclcpp::shutdown();
    return 0;
}