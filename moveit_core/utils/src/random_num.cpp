#include "moveit/utils/random.h"
#include <locale>

RandomGenerator::RandomGenerator() 
    : engine(std::random_device{}()) {}

RandomGenerator::RandomGenerator(unsigned int seed) 
    : engine(seed) {}

int RandomGenerator::getInt(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(engine);
}

double RandomGenerator::getDouble(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(engine);
}
