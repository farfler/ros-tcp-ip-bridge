#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                           // RCLCPP_DEBUG
#include "geometry_msgs/msg/twist_with_covariance.hpp" // geometry_msgs::msg::TwistWithCovariance

#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist_with_covariance.hpp" // GeometryMsgsMsgTwistWithCovariance
#include "tcp_ip_bridge/msgs/geometry_msgs/msg/twist.hpp"                 // GeometryMsgsMsgTwist

namespace tcp_ip_bridge
{

    std::vector<char> GeometryMsgsMsgTwistWithCovariance::serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> &msg)
    {
        std::vector<char> packet;

        GeometryMsgsMsgTwistWithCovariance::serialize(msg, packet);

        return packet;
    }

    std::vector<char> GeometryMsgsMsgTwistWithCovariance::serialize(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> &msg, std::vector<char> &packet)
    {
        GeometryMsgsMsgTwist::serialize(std::make_shared<geometry_msgs::msg::Twist>(msg->twist), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(msg->covariance.data()), reinterpret_cast<const char *>(msg->covariance.data() + msg->covariance.size() * sizeof(double)));

        return packet;
    }

    geometry_msgs::msg::TwistWithCovariance GeometryMsgsMsgTwistWithCovariance::deserialize(std::vector<char> &packet)
    {
        geometry_msgs::msg::TwistWithCovariance msg;

        GeometryMsgsMsgTwistWithCovariance::deserialize(packet, msg);

        return msg;
    }

    geometry_msgs::msg::TwistWithCovariance GeometryMsgsMsgTwistWithCovariance::deserialize(std::vector<char> &packet, geometry_msgs::msg::TwistWithCovariance &msg)
    {
        GeometryMsgsMsgTwist::deserialize(packet, msg.twist);

        memcpy(msg.covariance.data(), packet.data(), msg.covariance.size() * sizeof(double));
        packet.erase(packet.begin(), packet.begin() + msg.covariance.size() * sizeof(double));

        return msg;
    }

} // namespace tcp_ip_bridge