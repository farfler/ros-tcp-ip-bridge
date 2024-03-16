#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                           // RCLCPP_DEBUG
#include "geometry_msgs/msg/accel_with_covariance.hpp" // geometry_msgs::msg::AccelWithCovariance

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/accel_with_covariance.hpp" // GeometryMsgsMsgAccelWithCovariance
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/accel.hpp"                 // GeometryMsgsMsgAccel

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgAccelWithCovariance::serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovariance> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgAccelWithCovariance::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgAccelWithCovariance::serialize(const std::shared_ptr<geometry_msgs::msg::AccelWithCovariance> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgAccel::serialize(std::make_shared<geometry_msgs::msg::Accel>(msg->accel), packet);

        packet.insert(packet.end(), msg->covariance.data(), msg->covariance.data() + msg->covariance.size());

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_accel_with_covariance::serialize"), "covariance: %f", msg->covariance[0]);

        return packet;
    }

    geometry_msgs::msg::AccelWithCovariance GeometryMsgsMsgAccelWithCovariance::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::AccelWithCovariance msg;

        GeometryMsgsMsgAccelWithCovariance::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::AccelWithCovariance GeometryMsgsMsgAccelWithCovariance::deserialize(std::vector<char> &packet, geometry_msgs::msg::AccelWithCovariance &msg)
    {
        GeometryMsgsMsgAccel::deserialize(packet, msg.accel);

        memcpy(msg.covariance.data(), packet.data(), msg.covariance.size() * sizeof(double));
        packet.erase(packet.begin(), packet.begin() + msg.covariance.size() * sizeof(double));

        RCLCPP_DEBUG(rclcpp::get_logger("geometry_msgs_msg_accel_with_covariance::deserialize"), "covariance: %f", msg.covariance[0]);

        return msg;
    }

} // namespace tcp_ip_bridge