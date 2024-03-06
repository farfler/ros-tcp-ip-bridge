#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SensorMsgsMsgLaserScanTCPServer : public rclcpp::Node
{
public:
    SensorMsgsMsgLaserScanTCPServer() : Node("sensor_msgs_msg_laser_scan_tcp_server")
    {
    }

private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMsgsMsgLaserScanTCPServer>());
    rclcpp::shutdown();
    return 0;
}