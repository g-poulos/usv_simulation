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

int findClosest(float arr[], int n, float target);

void readAreaFile(float* &angle_table_ptr, float* &area_table_ptr);
#endif