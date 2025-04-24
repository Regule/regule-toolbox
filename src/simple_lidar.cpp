#include "regule-toolbox/simple_lidar.hpp"

namespace rgltool
{
    std::string LidarMeasurement::str() const
    {
        std::stringstream text;
        text << "State:" << state_to_str(state) << " ";
        text << "Condition:" << (condition ? "TRUE" : "FALSE") << " ";
        text << "Angle:" << std::setprecision(3) << angle << " ";
        text << "Distance:" << std::setprecision(3) << distance << " ";
        std::string str = text.str();
        return str;
    }

    std::string LidarMeasurement::state_to_str(State state)
    {
        switch (state)
        {
        case OK:
            return std::string("OK");
        case TIMEOUT:
            return std::string("TIMEOUT");
        case OUT_OF_RANGE:
            return std::string("OUT_OF_RANGE");
        case ERROR:
            return std::string("ERROR");
        default:
            return std::string("WTF");
        }
    }
}