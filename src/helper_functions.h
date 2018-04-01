#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

#define N_SAMPLE 10
#define DT 0.1


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Reads waypoints from a file.
 *
 * @param filename Name of file containing waypoint data data.
 * @output True if opening and reading file was successful
 */
inline bool read_waypoint_data(std::string filename, std::vector<double>& waypoint_x, std::vector<double>& waypoint_y) {

  // Get file of map:
  std::ifstream in_file(filename.c_str(),std::ifstream::in);
  // Return if we can't open the file.
  if (!in_file) {
      return false;
  }

  // Declare single line of map file:
  std::string line_waypoint;

  // skip header
  getline(in_file, line_waypoint);

  // Run over each single line:
  std::string cell;
  while(getline(in_file, line_waypoint)){
      std::istringstream iss_waypoint(line_waypoint);

      getline(iss_waypoint, cell, ',');
      double x = std::stod(cell);
      waypoint_x.push_back(x);
      getline(iss_waypoint, cell, ',');
      double y = std::stod(cell);
      waypoint_y.push_back(y);
  }
  return true;
}

#endif