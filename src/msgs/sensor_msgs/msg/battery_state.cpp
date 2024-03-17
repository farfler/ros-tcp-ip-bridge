#include <vector>       // std::vector
#include <cstring>      // memcpy
#include <netinet/in.h> // htonl and ntohl

#include "rclcpp/rclcpp.hpp"                 // RCLCPP_DEBUG
#include "sensor_msgs/msg/battery_state.hpp" // sensor_msgs::msg::BatteryState

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/battery_state.hpp" // SensorMsgsMsgBatteryState
#include "tcp_ip_bridge/msgs/std_msgs/msg/header.hpp"           // StdMsgsMsgHeader

namespace tcp_ip_bridge
{

    std::vector<char> SensorMsgsMsgBatteryState::serialize(const std::shared_ptr<sensor_msgs::msg::BatteryState> &msg)
    {
        std::vector<char> packet;

        SensorMsgsMsgBatteryState::serialize(msg, packet);

        return packet;
    }

    std::vector<char> SensorMsgsMsgBatteryState::serialize(const std::shared_ptr<sensor_msgs::msg::BatteryState> &msg, std::vector<char> &packet)
    {
        StdMsgsMsgHeader::serialize(std::make_shared<std_msgs::msg::Header>(msg->header), packet);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->voltage), reinterpret_cast<const char *>(&msg->voltage) + sizeof(msg->voltage));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "voltage: %f", msg->voltage);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->temperature), reinterpret_cast<const char *>(&msg->temperature) + sizeof(msg->temperature));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "temperature: %f", msg->temperature);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->current), reinterpret_cast<const char *>(&msg->current) + sizeof(msg->current));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "current: %f", msg->current);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->charge), reinterpret_cast<const char *>(&msg->charge) + sizeof(msg->charge));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "charge: %f", msg->charge);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->capacity), reinterpret_cast<const char *>(&msg->capacity) + sizeof(msg->capacity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "capacity: %f", msg->capacity);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->design_capacity), reinterpret_cast<const char *>(&msg->design_capacity) + sizeof(msg->design_capacity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "design_capacity: %f", msg->design_capacity);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->percentage), reinterpret_cast<const char *>(&msg->percentage) + sizeof(msg->percentage));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "percentage: %f", msg->percentage);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->power_supply_status), reinterpret_cast<const char *>(&msg->power_supply_status) + sizeof(msg->power_supply_status));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "power_supply_status: %d", msg->power_supply_status);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->power_supply_health), reinterpret_cast<const char *>(&msg->power_supply_health) + sizeof(msg->power_supply_health));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "power_supply_health: %d", msg->power_supply_health);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->power_supply_technology), reinterpret_cast<const char *>(&msg->power_supply_technology) + sizeof(msg->power_supply_technology));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "power_supply_technology: %d", msg->power_supply_technology);

        packet.insert(packet.end(), reinterpret_cast<const char *>(&msg->present), reinterpret_cast<const char *>(&msg->present) + sizeof(msg->present));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "present: %d", msg->present);

        uint32_t cell_voltage_size = htonl(static_cast<uint32_t>(msg->cell_voltage.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&cell_voltage_size), reinterpret_cast<const char *>(&cell_voltage_size) + sizeof(cell_voltage_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "cell_voltage_size: %u", ntohl(cell_voltage_size));

        if (msg->cell_voltage.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->cell_voltage.data()), reinterpret_cast<const char *>(msg->cell_voltage.data() + msg->cell_voltage.size()));
        }

        uint32_t cell_temperature_size = htonl(static_cast<uint32_t>(msg->cell_temperature.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&cell_temperature_size), reinterpret_cast<const char *>(&cell_temperature_size) + sizeof(cell_temperature_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "cell_temperature_size: %u", ntohl(cell_temperature_size));

        if (msg->cell_temperature.size() > 0)
        {
            packet.insert(packet.end(), reinterpret_cast<const char *>(msg->cell_temperature.data()), reinterpret_cast<const char *>(msg->cell_temperature.data() + msg->cell_temperature.size()));
        }

        uint32_t location_size = htonl(static_cast<uint32_t>(msg->location.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&location_size), reinterpret_cast<const char *>(&location_size) + sizeof(location_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "location_size: %u", ntohl(location_size));

        if (msg->location.size() > 0)
        {
            packet.insert(packet.end(), msg->location.data(), msg->location.data() + msg->location.size());

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "location: %s", msg->location.c_str());
        }

        uint32_t serial_number_size = htonl(static_cast<uint32_t>(msg->serial_number.size()));
        packet.insert(packet.end(), reinterpret_cast<const char *>(&serial_number_size), reinterpret_cast<const char *>(&serial_number_size) + sizeof(serial_number_size));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "serial_number_size: %u", ntohl(serial_number_size));

        if (msg->serial_number.size() > 0)
        {
            packet.insert(packet.end(), msg->serial_number.data(), msg->serial_number.data() + msg->serial_number.size());

            RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::serialize"), "serial_number: %s", msg->serial_number.c_str());
        }

        return packet;
    }

    sensor_msgs::msg::BatteryState SensorMsgsMsgBatteryState::deserialize(std::vector<char> &packet)
    {
        sensor_msgs::msg::BatteryState msg;

        SensorMsgsMsgBatteryState::deserialize(packet, msg);

        return msg;
    }

    sensor_msgs::msg::BatteryState SensorMsgsMsgBatteryState::deserialize(std::vector<char> &packet, sensor_msgs::msg::BatteryState &msg)
    {
        StdMsgsMsgHeader::deserialize(packet, msg.header);

        memcpy(&msg.voltage, packet.data(), sizeof(msg.voltage));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.voltage));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "voltage: %f", msg.voltage);

        memcpy(&msg.temperature, packet.data(), sizeof(msg.temperature));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.temperature));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "temperature: %f", msg.temperature);

        memcpy(&msg.current, packet.data(), sizeof(msg.current));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.current));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "current: %f", msg.current);

        memcpy(&msg.charge, packet.data(), sizeof(msg.charge));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.charge));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "charge: %f", msg.charge);

        memcpy(&msg.capacity, packet.data(), sizeof(msg.capacity));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.capacity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "capacity: %f", msg.capacity);

        memcpy(&msg.design_capacity, packet.data(), sizeof(msg.design_capacity));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.design_capacity));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "design_capacity: %f", msg.design_capacity);

        memcpy(&msg.percentage, packet.data(), sizeof(msg.percentage));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.percentage));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "percentage: %f", msg.percentage);

        memcpy(&msg.power_supply_status, packet.data(), sizeof(msg.power_supply_status));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.power_supply_status));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "power_supply_status: %d", msg.power_supply_status);

        memcpy(&msg.power_supply_health, packet.data(), sizeof(msg.power_supply_health));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.power_supply_health));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "power_supply_health: %d", msg.power_supply_health);

        memcpy(&msg.power_supply_technology, packet.data(), sizeof(msg.power_supply_technology));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.power_supply_technology));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "power_supply_technology: %d", msg.power_supply_technology);

        memcpy(&msg.present, packet.data(), sizeof(msg.present));
        packet.erase(packet.begin(), packet.begin() + sizeof(msg.present));

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "present: %d", msg.present);

        uint32_t cell_voltage_size;
        memcpy(&cell_voltage_size, packet.data(), sizeof(cell_voltage_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(cell_voltage_size));
        cell_voltage_size = ntohl(cell_voltage_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "cell_voltage_size: %u", cell_voltage_size);

        if (cell_voltage_size > 0)
        {
            msg.cell_voltage.resize(cell_voltage_size);
            memcpy(msg.cell_voltage.data(), packet.data(), cell_voltage_size * sizeof(float));
            packet.erase(packet.begin(), packet.begin() + cell_voltage_size * sizeof(float));
        }

        uint32_t cell_temperature_size;
        memcpy(&cell_temperature_size, packet.data(), sizeof(cell_temperature_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(cell_temperature_size));
        cell_temperature_size = ntohl(cell_temperature_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "cell_temperature_size: %u", cell_temperature_size);

        if (cell_temperature_size > 0)
        {
            msg.cell_temperature.resize(cell_temperature_size);
            memcpy(msg.cell_temperature.data(), packet.data(), cell_temperature_size * sizeof(float));
            packet.erase(packet.begin(), packet.begin() + cell_temperature_size * sizeof(float));
        }

        uint32_t location_size;
        memcpy(&location_size, packet.data(), sizeof(location_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(location_size));
        location_size = ntohl(location_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "location_size: %u", location_size);

        if (location_size > 0)
        {
            msg.location.resize(location_size);
            memcpy(msg.location.data(), packet.data(), location_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + location_size * sizeof(char));
        }

        uint32_t serial_number_size;
        memcpy(&serial_number_size, packet.data(), sizeof(serial_number_size));
        packet.erase(packet.begin(), packet.begin() + sizeof(serial_number_size));
        serial_number_size = ntohl(serial_number_size);

        RCLCPP_DEBUG(rclcpp::get_logger("sensor_msgs_msg_battery_state::deserialize"), "serial_number_size: %u", serial_number_size);

        if (serial_number_size > 0)
        {
            msg.serial_number.resize(serial_number_size);
            memcpy(msg.serial_number.data(), packet.data(), serial_number_size * sizeof(char));
            packet.erase(packet.begin(), packet.begin() + serial_number_size * sizeof(char));
        }

        return msg;
    }

} // namespace tcp_ip_bridge