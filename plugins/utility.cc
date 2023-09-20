#include "utility.hh"
#include <random>

GaussianNoise::GaussianNoise() {
    this->distribution = std::normal_distribution<double> (0, 1.0);
}

GaussianNoise::GaussianNoise(double mean, double stddev) {
    this->distribution = std::normal_distribution<double> (mean, stddev);
}

double GaussianNoise::getNoise() {
        std::random_device rd;
        std::mt19937 gen(rd());
        return this->distribution(gen);
}
