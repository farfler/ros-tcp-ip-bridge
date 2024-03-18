#ifndef TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__FLUID_PRESSURE_HPP_
#define TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__FLUID_PRESSURE_HPP_

#include <vector> // std::vector

#include "sensor_msgs/msg/fluid_pressure.hpp" // sensor_msgs::msg::FluidPressure

namespace tcp_ip_bridge
{

    class SensorMsgsMsgFluidPressure
    {
    public:
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::FluidPressure> &msg);
        static std::vector<char> serialize(const std::shared_ptr<sensor_msgs::msg::FluidPressure> &msg, std::vector<char> &packet);
        static sensor_msgs::msg::FluidPressure deserialize(std::vector<char> &packet);
        static sensor_msgs::msg::FluidPressure deserialize(std::vector<char> &packet, sensor_msgs::msg::FluidPressure &msg);
    };

} // namespace tcp_ip_bridge

#endif // TCP_IP_BRIDGE__MSGS__SENSOR_MSGS__MSG__FLUID_PRESSURE_HPP_