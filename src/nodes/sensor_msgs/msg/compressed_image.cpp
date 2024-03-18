#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "tcp_ip_bridge/msgs/sensor_msgs/msg/compressed_image.hpp"
#include "tcp_ip_bridge/templates/tcp/client.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericTCPClient<sensor_msgs::msg::CompressedImage, SensorMsgsMsgCompressedImage>>("sensor_msgs_msg_compressed_image_tcp_client"));
    rclcpp::shutdown();
    return 0;
}
