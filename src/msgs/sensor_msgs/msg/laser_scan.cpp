#include <vector>
#include <string>
#include <cstring>

#include "tcp_ip_bridge/msgs/sensor_msgs/msg/laser_scan.hpp"

namespace tcp_ip_bridge
{

    void serialize_sensor_msgs_msg_laser_scan(const SensorMsgsMsgLaserScan &msg, std::vector<char> &buffer)
    {
        buffer.clear();
        uint32_t offset = 0;

        // std_msgs/msg/Header frame_id size
        uint32_t header_frame_id_size = static_cast<uint32_t>(msg.header_frame_id.size());
        buffer.resize(offset + sizeof(header_frame_id_size));
        memcpy(buffer.data() + offset, &header_frame_id_size, sizeof(header_frame_id_size));
        offset += sizeof(header_frame_id_size);

        // std_msgs/msg/Header frame_id data
        buffer.resize(offset + header_frame_id_size);
        memcpy(buffer.data() + offset, msg.header_frame_id.data(), header_frame_id_size);
        offset += header_frame_id_size;

        // builtin_interfaces/msg/Time sec
        buffer.resize(offset + sizeof(msg.header_stamp_sec));
        memcpy(buffer.data() + offset, &msg.header_stamp_sec, sizeof(msg.header_stamp_sec));
        offset += sizeof(msg.header_stamp_sec);

        // builtin_interfaces/msg/Time nanosec
        buffer.resize(offset + sizeof(msg.header_stamp_nanosec));
        memcpy(buffer.data() + offset, &msg.header_stamp_nanosec, sizeof(msg.header_stamp_nanosec));
        offset += sizeof(msg.header_stamp_nanosec);

        // sensor_msgs/msg/LaserScan angle_min
        buffer.resize(offset + sizeof(msg.angle_min));
        memcpy(buffer.data() + offset, &msg.angle_min, sizeof(msg.angle_min));
        offset += sizeof(msg.angle_min);

        // sensor_msgs/msg/LaserScan angle_max
        buffer.resize(offset + sizeof(msg.angle_max));
        memcpy(buffer.data() + offset, &msg.angle_max, sizeof(msg.angle_max));
        offset += sizeof(msg.angle_max);

        // sensor_msgs/msg/LaserScan angle_increment
        buffer.resize(offset + sizeof(msg.angle_increment));
        memcpy(buffer.data() + offset, &msg.angle_increment, sizeof(msg.angle_increment));
        offset += sizeof(msg.angle_increment);

        // sensor_msgs/msg/LaserScan time_increment
        buffer.resize(offset + sizeof(msg.time_increment));
        memcpy(buffer.data() + offset, &msg.time_increment, sizeof(msg.time_increment));
        offset += sizeof(msg.time_increment);

        // sensor_msgs/msg/LaserScan scan_time
        buffer.resize(offset + sizeof(msg.scan_time));
        memcpy(buffer.data() + offset, &msg.scan_time, sizeof(msg.scan_time));
        offset += sizeof(msg.scan_time);

        // sensor_msgs/msg/LaserScan range_min
        buffer.resize(offset + sizeof(msg.range_min));
        memcpy(buffer.data() + offset, &msg.range_min, sizeof(msg.range_min));
        offset += sizeof(msg.range_min);

        // sensor_msgs/msg/LaserScan range_max
        buffer.resize(offset + sizeof(msg.range_max));
        memcpy(buffer.data() + offset, &msg.range_max, sizeof(msg.range_max));
        offset += sizeof(msg.range_max);

        // sensor_msgs/msg/LaserScan ranges size
        uint32_t ranges_size = static_cast<uint32_t>(msg.ranges.size());
        buffer.resize(offset + sizeof(ranges_size));
        memcpy(buffer.data() + offset, &ranges_size, sizeof(ranges_size));
        offset += sizeof(ranges_size);

        // sensor_msgs/msg/LaserScan ranges data
        buffer.resize(offset + ranges_size * sizeof(float));
        memcpy(buffer.data() + offset, msg.ranges.data(), ranges_size * sizeof(float));
        offset += ranges_size * sizeof(float);

        // sensor_msgs/msg/LaserScan intensities size
        uint32_t intensities_size = static_cast<uint32_t>(msg.intensities.size());
        buffer.resize(offset + sizeof(intensities_size));
        memcpy(buffer.data() + offset, &intensities_size, sizeof(intensities_size));
        offset += sizeof(intensities_size);

        // sensor_msgs/msg/LaserScan intensities data
        buffer.resize(offset + intensities_size * sizeof(float));
        memcpy(buffer.data() + offset, msg.intensities.data(), intensities_size * sizeof(float));
        offset += intensities_size * sizeof(float);
    }

    void deserialize_sensor_msgs_msg_laser_scan(const std::vector<char> &buffer, SensorMsgsMsgLaserScan &msg)
    {
        uint32_t offset = 0;

        // std_msgs/msg/Header frame_id size
        uint32_t header_frame_id_size;
        memcpy(&header_frame_id_size, buffer.data() + offset, sizeof(header_frame_id_size));
        offset += sizeof(header_frame_id_size);

        // std_msgs/msg/Header frame_id data
        msg.header_frame_id.assign(buffer.data() + offset, header_frame_id_size);
        offset += header_frame_id_size;

        // builtin_interfaces/msg/Time sec
        memcpy(&msg.header_stamp_sec, buffer.data() + offset, sizeof(msg.header_stamp_sec));
        offset += sizeof(msg.header_stamp_sec);

        // builtin_interfaces/msg/Time nanosec
        memcpy(&msg.header_stamp_nanosec, buffer.data() + offset, sizeof(msg.header_stamp_nanosec));
        offset += sizeof(msg.header_stamp_nanosec);

        // sensor_msgs/msg/LaserScan angle_min
        memcpy(&msg.angle_min, buffer.data() + offset, sizeof(msg.angle_min));
        offset += sizeof(msg.angle_min);

        // sensor_msgs/msg/LaserScan angle_max
        memcpy(&msg.angle_max, buffer.data() + offset, sizeof(msg.angle_max));
        offset += sizeof(msg.angle_max);

        // sensor_msgs/msg/LaserScan angle_increment
        memcpy(&msg.angle_increment, buffer.data() + offset, sizeof(msg.angle_increment));
        offset += sizeof(msg.angle_increment);

        // sensor_msgs/msg/LaserScan time_increment
        memcpy(&msg.time_increment, buffer.data() + offset, sizeof(msg.time_increment));
        offset += sizeof(msg.time_increment);

        // sensor_msgs/msg/LaserScan scan_time
        memcpy(&msg.scan_time, buffer.data() + offset, sizeof(msg.scan_time));
        offset += sizeof(msg.scan_time);

        // sensor_msgs/msg/LaserScan range_min
        memcpy(&msg.range_min, buffer.data() + offset, sizeof(msg.range_min));
        offset += sizeof(msg.range_min);

        // sensor_msgs/msg/LaserScan range_max
        memcpy(&msg.range_max, buffer.data() + offset, sizeof(msg.range_max));
        offset += sizeof(msg.range_max);

        // sensor_msgs/msg/LaserScan ranges size
        uint32_t ranges_size;
        memcpy(&ranges_size, buffer.data() + offset, sizeof(ranges_size));
        offset += sizeof(ranges_size);

        // sensor_msgs/msg/LaserScan ranges data
        msg.ranges.resize(ranges_size);
        memcpy(msg.ranges.data(), buffer.data() + offset, ranges_size * sizeof(float));
        offset += ranges_size * sizeof(float);

        // sensor_msgs/msg/LaserScan intensities size
        uint32_t intensities_size;
        memcpy(&intensities_size, buffer.data() + offset, sizeof(intensities_size));
        offset += sizeof(intensities_size);

        // sensor_msgs/msg/LaserScan intensities data
        msg.intensities.resize(intensities_size);
        memcpy(msg.intensities.data(), buffer.data() + offset, intensities_size * sizeof(float));
        offset += intensities_size * sizeof(float);
    }

} // namespace tcp_ip_bridge