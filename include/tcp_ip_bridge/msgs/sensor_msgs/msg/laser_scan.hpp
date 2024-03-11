#include <vector>
#include <string>

namespace tcp_ip_bridge
{

    /**
     * @brief `SensorMsgsMsgLaserScan` message. Extracted from `sensor_msgs/msg/LaserScan`.
     */
    struct SensorMsgsMsgLaserScan
    {
        std::string header_frame_id;
        int32_t header_stamp_sec;
        uint32_t header_stamp_nanosec;
        float angle_min;
        float angle_max;
        float angle_increment;
        float time_increment;
        float scan_time;
        float range_min;
        float range_max;
        std::vector<float> ranges;
        std::vector<float> intensities;
    };

    /**
     * @brief Serialize `SensorMsgsMsgLaserScan` message into a `std::vector<char>`.
     * @param msg `SensorMsgsMsgLaserScan` message to serialize.
     * @param buffer `std::vector<char>` buffer to serialize into.
     */
    void serialize_sensor_msgs_msg_laser_scan(const SensorMsgsMsgLaserScan &msg, std::vector<char> &buffer);

    /**
     * @brief Deserialize `std::vector<char>` into a `SensorMsgsMsgLaserScan` message.
     * @param buffer `std::vector<char>` buffer to deserialize from.
     * @param msg `SensorMsgsMsgLaserScan` message to deserialize.
     */
    void deserialize_sensor_msgs_msg_laser_scan(const std::vector<char> &buffer, SensorMsgsMsgLaserScan &msg);

} // namespace tcp_ip_bridge