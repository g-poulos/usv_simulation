#ifndef UTILITY_H
#define UTILITY_H

#include <random>
#include <gz/math/Vector3.hh>
#include <optional>
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"

using namespace gz;

struct wrenchDataStruct{
    int size;
    float *angle;
    float *forceArea;
    float *torqueArea;
    float *offset;
};

struct wrenchStruct{
    math::Vector3d force;
    math::Vector3d torque;
};

typedef struct wrenchDataStruct wrenchFileData;
typedef struct wrenchStruct wrenchData;

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
                         double initValue, double dt);
    double getValue();

};

int findClosestMatchingValue(float arr[], int n, float target);

wrenchFileData* read_csv(std::string filename);

std::string findFileFromHome(const std::string& filename);

std::string getModelFile(sim::EntityComponentManager &_ecm, std::string fileName);

math::Vector3d sphericalToVector(double magnitude, double elevation, double azimuth);

math::Vector3d toGZVec(std::optional<math::Vector3<double>> vec);

wrenchData calculateWrench(sim::EntityComponentManager &_ecm, sim::Link link, float speed, float direction,
                           wrenchFileData *surfaceData, float fluidDensity, float resCoefficient);
#endif