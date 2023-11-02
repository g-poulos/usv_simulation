#include "utility.hh"
#include <random>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string.h>
#include <filesystem>

namespace fs = std::filesystem;
using namespace std;


IntegratedWhiteNoise::IntegratedWhiteNoise() {
    this->distribution = std::normal_distribution<double> (0, 1);
    this->minValue = 1;
    this->maxValue = 2;
    this->dt = 0.01;
    this->prevValue = 0.5 * (this->minValue + this->maxValue);

}

IntegratedWhiteNoise::IntegratedWhiteNoise(double mean, double stddev,
                                           double minValue, double maxValue,
                                           double dt) {
    this->distribution = std::normal_distribution<double> (mean, stddev);
    this->minValue = minValue;
    this->maxValue = maxValue;
    this->dt = dt;
    this->prevValue = 0.5 * (maxValue + minValue);
}

double IntegratedWhiteNoise::getValue() {
    std::random_device rd;
    std::mt19937 gen(rd());
    double whiteNoise = this->distribution(gen);
    double nextValue = prevValue + this->dt * whiteNoise;

    if (nextValue > this->maxValue || nextValue < this->minValue) {
        nextValue = nextValue - this->dt * whiteNoise;
    }

    this->prevValue = nextValue;
    return nextValue;
}

int getClosest(float val1, float val2, float target)
{
    if (target - val1 >= val2 - target)
        // pointer for val2;
        return 1;
    else
        // pointer for val1;
        return 0;
}


// Returns index to element closest to target in arr[]
int findClosest(float arr[], int n, float target) {
    // Corner cases
    if (target <= arr[0])
        return 0;
    if (target >= arr[n - 1])
        return n - 1;

    // Binary search
    int i = 0, j = n, mid = 0;
    while (i < j) {
        mid = (i + j) / 2;

        if (arr[mid] == target)
            return mid;

        if (target < arr[mid]) {
            if (mid > 0 && target > arr[mid - 1])
                return mid - 1 + getClosest(arr[mid], arr[mid] + 1, target);
            j = mid;
        }
            /* Repeat for left half */

        else {
            if (mid < n - 1 && target < arr[mid + 1])
                return mid + getClosest(arr[mid], arr[mid] + 1, target);
            i = mid + 1;
        }
    }
    return mid;
}

void readAreaFile(std::string filename, float* &angle_table_ptr, float* &area_table_ptr) {
    std::filesystem::path cwd = std::filesystem::current_path();
    cout << "DIRECTORY: " << cwd << std::endl;
    string myText;
    ifstream MyReadFile(filename);
    getline(MyReadFile, myText);
    int arraySize = std::stoi(myText);

    angle_table_ptr = new float[arraySize];
    area_table_ptr = new float[arraySize];

    bool foundSymb = false;
    int i = 0;
    const char * c2 = "#";

    while (getline (MyReadFile, myText)) {
        char *c = myText.data();
        float value;
        if (strcmp(c, c2) == 0) {
            foundSymb = true;
            i = 0;
        } else {
            value = std::stof(myText);
            if (foundSymb) {
                area_table_ptr[i] = value;
            } else {
                angle_table_ptr[i] = value;
            }
        }
        i++;
    }
    MyReadFile.close();
}

std::string findFileFromHome(const std::string& filename) {
    fs::path homeDir = fs::path(getenv("HOME")); // Get the home directory

    for (const auto& entry : fs::recursive_directory_iterator(homeDir)) {
        if (entry.is_regular_file() && entry.path().filename() == filename) {
            return entry.path();
        }
    }
    return ""; // Empty string indicates file not found
}
