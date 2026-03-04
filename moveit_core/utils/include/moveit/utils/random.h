#pragma once
#include <random>

class RandomGenerator {
public:
    RandomGenerator();
    RandomGenerator(unsigned int seed);

    int getInt(int min, int max);
    double getDouble(double min, double max);

private:
    std::mt19937 engine;
};
