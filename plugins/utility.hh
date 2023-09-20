#ifndef UTILITY_H
#define UTILITY_H

#include <random>

class GaussianNoise {
private:
    std::normal_distribution<double> distribution;

public:
    GaussianNoise();
    GaussianNoise(double mean, double stddev);
    double getNoise();
};

#endif