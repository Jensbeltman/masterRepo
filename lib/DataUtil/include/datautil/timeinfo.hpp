#ifndef MASTER_TIMEINFO_HPP
#define MASTER_TIMEINFO_HPP
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

std::string getTimeString(std::string fmt)
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, fmt.c_str());
    auto str = oss.str();

    return str;
}

#endif //MASTER_TIMEINFO_HPP
