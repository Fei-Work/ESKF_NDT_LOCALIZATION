#ifndef TOOL_H
#define TOOL_H
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>

namespace tool
{
    std::vector<double> readCsvColumn(const std::string& filename, int columnNumber) {
        std::vector<double> columnData;
        std::ifstream file(filename);
        if (file.is_open()) {
            std::string line;
            while (getline(file, line)) {
                std::stringstream ss(line);
                std::string token;
                int currentColumn = 0;
                // 逐列解析每一行
                while (getline(ss, token, ',')) {
                    if (currentColumn == columnNumber) {
                        // 将解析的字符串转换为 double 类型
                        double value = std::stod(token);
                        columnData.push_back(value);
                        break;
                    }
                    currentColumn++;
                }
            }
            file.close();
        } else {
            std::cerr << "Unable to open file: " << filename << std::endl;
        }
        return columnData;
    }


    std::filesystem::path GetCurrentSourceDirectory() {
        std::filesystem::path filePath = std::filesystem::canonical(__FILE__);
        return filePath.parent_path();
    }
    
} // namespace name
#endif