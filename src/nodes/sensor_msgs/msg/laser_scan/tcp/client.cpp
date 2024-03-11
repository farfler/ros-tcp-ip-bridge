#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SensorMsgsMsgLaserScanTCPClient : public rclcpp::Node
{
public:
    SensorMsgsMsgLaserScanTCPClient() : Node("sensor_msgs_msg_laser_scan_tcp_client")
    {
    }

private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMsgsMsgLaserScanTCPClient>());
    rclcpp::shutdown();
    return 0;
}