#include <cstdlib>
#include <list>
#include <Eigen/Core>
#include "Trajectory.h"

int numTests = 0;
int numTestsFailed = 0;

void test(std::string name, std::list<Eigen::VectorXd> waypoints, double maxDeviation, const Eigen::VectorXd& maxVelocities, const Eigen::VectorXd& maxAccelerations, double timeStep) {
  Trajectory trajectory(Path(waypoints, maxDeviation), maxVelocities, maxAccelerations, timeStep);
  numTests++;
  if(!trajectory.isValid()) {
    numTestsFailed++;
    std::cout << name << " with time step " << timeStep << " failed." << std::endl;
    trajectory.outputPhasePlaneTrajectory();
  }
}

void test1() {
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
  waypoints.push_back(waypoint);
  waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd maxVelocities(4);
  maxVelocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd maxAccelerations(4);
  maxAccelerations << 0.00249, 0.00249, 0.00249, 0.00249;

  test("Test 1", waypoints, 100.0, maxVelocities, maxAccelerations, 10.0);
}

void test2() {
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd maxVelocities(4);
  maxVelocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd maxAccelerations(4);
  maxAccelerations << 0.002, 0.002, 0.002, 0.002;

  test("Test 2", waypoints, 100.0, maxVelocities, maxAccelerations, 10.0);
}

static inline double randomInRange(double min, double max) {
  return min + (max-min) * ((double)rand() / (double)RAND_MAX);
}

Eigen::VectorXd sampleConfig() {
  Eigen::VectorXd maxPositions(7);
  maxPositions << 115.0, 105.0, 90.0, 70.0, 130.0, 90.0, 55.0;
  maxPositions *= 3.14159 / 180.0;
  Eigen::VectorXd config(7);
  for(unsigned int i = 0; i < 7; i++) {
    config[i] = randomInRange(-maxPositions[i], maxPositions[i]);
  }
  return config;
}

void randomTests() {
  const int numIterations = 10000;

  Eigen::VectorXd maxVelocities(7);
  maxVelocities << 1.309, 1.6406, 3.2812, 2.618, 5.236, 3.4907, 3.4907;
  Eigen::VectorXd maxAccelerations = maxVelocities;

  for(int iteration = 0; iteration < numIterations; iteration++) {
    std::list<Eigen::VectorXd> waypoints;
    for(int numWaypoints = 0; numWaypoints < 10; numWaypoints++) {
      waypoints.push_back(sampleConfig());
    }
    std::stringstream name;
    name << "Random test " << iteration;
    test(name.str(), waypoints, 0.5, maxVelocities, maxAccelerations, 0.001);
    test(name.str(), waypoints, 0.5, maxVelocities, maxAccelerations, 0.01);
  }
}

int main() {
  test1();
  test2();
  randomTests();

  std::cout << numTestsFailed << " out of " << numTests << " tests failed." << std::endl;
  return 0;
}
