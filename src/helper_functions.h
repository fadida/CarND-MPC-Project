#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <iostream>
#include <thread>
#include <vector>
#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class Helper {
public:
    static constexpr double pi() { return M_PI; }

    // Convert degrees to radians.
    static double deg2rad(double x) { return x * pi() / 180; }

    // Convert radians to degrees.
    static double rad2deg(double x) { return x * 180 / pi(); }

    // Convert milliseconds to seconds.
    static double millisec2sec(double milli) { return milli / 1000.0; }

    // Convert mile per hour to Meter per hour.
    static double mph2mps(double mph) { return mph * 0.44704; }
};

#endif