#ifndef DIFFDRIVE_ARDUINO_DRIVE_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_DRIVE_WHEEL_HPP

#include <string>
#include <cmath>


class DriveWheel
{
    public:

    std::string name = "";
    double cmd = 0;
    double vel = 0;
    double pos = 0;

    DriveWheel() = default;

    DriveWheel(const std::string &wheel_name)
    {
      setup(wheel_name);
    }

    
    void setup(const std::string &wheel_name)
    {
      name = wheel_name;
    }




};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
