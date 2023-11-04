#ifndef UTILITY_H
#define UTILITY_H

#include <random>
#include <gz/math/Vector3.hh>
#include <optional>
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"

using namespace gz;

struct surfaceDataStruct{
    int size;
    float *area_p;
    float *angle_p;
};


typedef struct surfaceDataStruct surfaceData;


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

int findClosestMatchingValue(float arr[], int n, float target);

surfaceData* readAreaFile(std::string filename);

std::string findFileFromHome(const std::string& filename);

std::string getModelFile(sim::EntityComponentManager &_ecm, std::string fileName);

math::Vector3d sphericalToVector(double magnitude, double elevation, double azimuth);

math::Vector3d toGZVec(std::optional<math::Vector3<double>> vec);

float getSurface(sim::Link link, sim::EntityComponentManager &_ecm, float azimuth, surfaceData* surfaceData);

math::Vector3d calculateForce(sim::EntityComponentManager &_ecm,
                              sim::Link link,
                              float speed,
                              float direction,
                              surfaceData* surfaceData,
                              float fluidDensity);
#endif