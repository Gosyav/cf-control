#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "uav_dynamics/flatness_mapper.hpp"

double toDouble(const std::string & s) { return s.empty() ? 0.0 : std::stod(s); }

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "Usage: ros2 run uav_dynamics test_csv <file.csv>\n";
    return 1;
  }

  std::ifstream file(argv[1]);
  std::string line;

  std::getline(file, line);  // header

  FlatnessMapper mapper;

  int total = 0;
  int pass = 0;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> cols;

    while (std::getline(ss, cell, ',')) {
      cols.push_back(cell);
    }

    FlatInput in;
    int i = 1;

    in.pos = {toDouble(cols[i++]), toDouble(cols[i++]), toDouble(cols[i++])};
    in.vel = {toDouble(cols[i++]), toDouble(cols[i++]), toDouble(cols[i++])};
    in.acc = {toDouble(cols[i++]), toDouble(cols[i++]), toDouble(cols[i++])};
    in.jerk = {toDouble(cols[i++]), toDouble(cols[i++]), toDouble(cols[i++])};
    in.snap = {toDouble(cols[i++]), toDouble(cols[i++]), toDouble(cols[i++])};

    in.yaw = toDouble(cols[i++]);
    in.yaw_rate = toDouble(cols[i++]);
    in.yaw_acceleration = toDouble(cols[i++]);

    in.mass = toDouble(cols[i++]);
    in.gravity = toDouble(cols[i++]);

    in.I_xx = toDouble(cols[i++]);
    in.I_yy = toDouble(cols[i++]);
    in.I_zz = toDouble(cols[i++]);

    FlatOutput out = mapper.map(in);

    double expected_thrust = toDouble(cols[37]);

    double err = fabs(out.thrust - expected_thrust);

    total++;

    if (err < 1e-3) {
      pass++;
      std::cout << "PASS\n";
    } else {
      std::cout << "FAIL: " << err << "\n";
    }
  }

  std::cout << "\nRESULT: " << pass << "/" << total << std::endl;
}