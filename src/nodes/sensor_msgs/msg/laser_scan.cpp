#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tcp_ip_bridge/msgs/sensor_msgs/msg/laser_scan.hpp"
#include "tcp_ip_bridge/templates/tcp/client.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericTCPClient<sensor_msgs::msg::LaserScan, SensorMsgsMsgLaserScan>>("sensor_msgs_msg_laser_scan_tcp_client"));
    rclcpp::shutdown();
    return 0;
}
