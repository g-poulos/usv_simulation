#include "utility.hh"
#include <random>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string.h>

using namespace std;

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

void readAreaFile(float* &angle_table_ptr, float* &area_table_ptr) {
    std::filesystem::path cwd = std::filesystem::current_path();
    cout << "DIRECTORY: " << cwd << std::endl;
    string myText;
    ifstream MyReadFile("angles.txt");
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