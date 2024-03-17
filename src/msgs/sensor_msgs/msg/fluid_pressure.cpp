#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                  // RCLCPP_DEBUG
#include "sensor_msgs/msg/fluid_pressure.hpp" // sensor_msgs::msg::FluidPressure

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/fluid_pressure.hpp" // SensorMsgsMsgFluidPressure
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"            // StdMsgsMsgHeader

namespace tcp_ip_bridge
{
    std::vector<char> SensorMsgsMsgFluidPressure::serialize(const std::shared_ptr<sensor_msgs::msg::FluidPressure> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgFluidPressure::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgFluidPressure::serialize(const std::shared_ptr<sensor_msgs::msg::FluidPressure> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->fluid_pressure), reinterpret_cast<const char *>(&msg->fluid_pressure) + sizeof(msg->fluid_pressure));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_fluid_pressure::serialize"), "fluid_pressure: %f", msg->fluid_pressure);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->variance), reinterpret_cast<const char *>(&msg->variance) + sizeof(msg->variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_fluid_pressure::serialize"), "variance: %f", msg->variance);

        return packet;
    }

    sensor_msgs::msg::FluidPressure SensorMsgsMsgFluidPressure::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::FluidPressure msg;

        SensorMsgsMsgFluidPressure::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::FluidPressure SensorMsgsMsgFluidPressure::deserialize(std::vector<char> &packet, sensor_msgs::msg::FluidPressure &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.fluid_pressure, packet.data(), sizeof(msg.fluid_pressure));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.fluid_pressure));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_fluid_pressure::deserialize"), "fluid_pressure: %f", msg.fluid_pressure);

        memcpy(&msg.variance, packet.data(), sizeof(msg.variance));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.variance));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_fluid_pressure::deserialize"), "variance: %f", msg.variance);

        return msg;
    }

} // namespace tcp_ip_bridge