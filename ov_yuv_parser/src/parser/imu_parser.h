#ifndef IMU_PARSER_H
#define IMU_PARSER_H

#include <vector>
#include <string>

struct IMUData {
    int index;
    long long curPts;
    long long timeStamp;
    double accX, accY, accZ;      // Accelerometer data
    double gyroX, gyroY, gyroZ;   // Gyroscope data
    long long accPts;
    int gyroTemp;
    int accTemp;
    
    // 用于时间戳比较
    bool operator<(const IMUData& other) const {
        return timeStamp < other.timeStamp;
    }
};

class IMUParser {
public:
    /**
     * @brief Parse IMU data from imu.txt file
     * @param imu_file Path to imu.txt
     * @return Vector of IMU data sorted by timestamp
     */
    std::vector<IMUData> parseIMUFile(const std::string& imu_file);
    
    /**
     * @brief Print IMU data
     * @param imu IMU data to print
     */
    void printIMUData(const IMUData& imu);
    
    /**
     * @brief Print IMU statistics
     * @param imu_data Vector of IMU data
     */
    void printIMUStats(const std::vector<IMUData>& imu_data);
};

#endif // IMU_PARSER_H
