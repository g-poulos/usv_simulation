#ifndef UTILITY_H
#define UTILITY_H

#include <random>
#include <gz/math/Vector3.hh>
#include <optional>
#include "gz/sim/Model.hh"

using namespace gz;

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

void readAreaFile(std::string filename, float* &angle_table_ptr, float* &area_table_ptr);

std::string findFileFromHome(const std::string& filename);

math::Vector3d sphericalToVector(double magnitude, double elevation, double azimuth);

math::Vector3d toGZVec(std::optional<math::Vector3<double>> vec);

std::string getModelFile(sim::EntityComponentManager &_ecm, std::string fileName);
#endif