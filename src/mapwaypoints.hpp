#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_

#include <fstream>
#include <iostream>
#include <vector>
#include "helper.hpp"
#include "json.hpp"

class MapWaypoints {
 public:
  MapWaypoints(std::string data_file);

  ~MapWaypoints(){};

  void addWaypoint(double x, double y, double s, double d_x, double d_y);
  int getNextWaypoint(double x, double y, double theta);
  int getClosestWaypoint(double x, double y);
  std::vector<double> frenetToXY(double x, double y);
  std::vector<double> xyToFrenet(double x, double y, double theta);

 private:
  void _populateLists(std::string data_file);
  std::vector<double> _map_x;
  std::vector<double> _map_y;
  std::vector<double> _map_s;
  std::vector<double> _map_dx;
  std::vector<double> _map_dy;
};

#endif  // WAYPOINT_HPP_
