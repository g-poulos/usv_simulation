#ifndef UTILITY_H
#define UTILITY_H

#include <random>


class IntegratedWhiteNoise {
private:
    std::normal_distribution<double> distribution;
    double minValue;
    double maxValue;
    double dt;
    double prevValue;
public:
    IntegratedWhiteNoise();
    IntegratedWhiteNoise(double mean, double stddev,
                         double minValue, double maxValue,
                         double dt);
    double getValue();

};

int findClosest(float arr[], int n, float target);

void readAreaFile(float* &angle_table_ptr, float* &area_table_ptr);
#endif