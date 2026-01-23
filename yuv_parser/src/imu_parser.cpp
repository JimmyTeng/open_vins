#include "imu_parser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>

std::vector<IMUData> IMUParser::parseIMUFile(const std::string& imu_file) {
    std::vector<IMUData> imu_data;
    std::ifstream file(imu_file);
    
    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open IMU file " << imu_file << std::endl;
        return imu_data;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        IMUData imu;
        // Parse format: index:X, curPts:Y, timeStamp:Z, accX:..., accY:..., accZ:..., 
        //               gyroX:..., gyroY:..., gyroZ:..., accPts:..., gyroTemp:..., accTemp:...
        
        // Use stringstream to parse comma-separated values
        std::istringstream iss(line);
        std::string token;
        
        while (std::getline(iss, token, ',')) {
            // Remove leading/trailing whitespace
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            
            // Parse key-value pairs
            size_t colon_pos = token.find(':');
            if (colon_pos == std::string::npos) continue;
            
            std::string key = token.substr(0, colon_pos);
            std::string value = token.substr(colon_pos + 1);
            
            if (key == "index") {
                imu.index = std::stoi(value);
            } else if (key == "curPts") {
                imu.curPts = std::stoll(value);
            } else if (key == "timeStamp") {
                imu.timeStamp = std::stoll(value);
            } else if (key == "accX") {
                imu.accX = std::stod(value);
            } else if (key == "accY") {
                imu.accY = std::stod(value);
            } else if (key == "accZ") {
                imu.accZ = std::stod(value);
            } else if (key == "gyroX") {
                imu.gyroX = std::stod(value);
            } else if (key == "gyroY") {
                imu.gyroY = std::stod(value);
            } else if (key == "gyroZ") {
                imu.gyroZ = std::stod(value);
            } else if (key == "accPts") {
                imu.accPts = std::stoll(value);
            } else if (key == "gyroTemp") {
                imu.gyroTemp = std::stoi(value);
            } else if (key == "accTemp") {
                imu.accTemp = std::stoi(value);
            }
        }
        
        imu_data.push_back(imu);
    }
    
    file.close();
    
    // Sort by timestamp
    std::sort(imu_data.begin(), imu_data.end());
    
    return imu_data;
}

void IMUParser::printIMUData(const IMUData& imu) {
    std::cout << "IMU[" << imu.index << "] timestamp=" << imu.timeStamp 
              << " acc=(" << std::fixed << std::setprecision(3) 
              << imu.accX << "," << imu.accY << "," << imu.accZ << ")"
              << " gyro=(" << imu.gyroX << "," << imu.gyroY << "," << imu.gyroZ << ")" << std::endl;
}

void IMUParser::printIMUStats(const std::vector<IMUData>& imu_data) {
    if (imu_data.empty()) {
        std::cout << "No IMU data" << std::endl;
        return;
    }
    
    std::cout << "IMU Statistics:" << std::endl;
    std::cout << "  Total entries: " << imu_data.size() << std::endl;
    std::cout << "  First timestamp: " << imu_data[0].timeStamp << std::endl;
    std::cout << "  Last timestamp: " << imu_data.back().timeStamp << std::endl;
    std::cout << "  Time range: " << (imu_data.back().timeStamp - imu_data[0].timeStamp) << " pts" << std::endl;
}
