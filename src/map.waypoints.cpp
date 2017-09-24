#include "map.waypoints.hpp"

using namespace std;

MapWaypoints::MapWaypoints() {
  // empty
}

MapWaypoints::MapWaypoints(string data_file) { _populateLists(data_file); }

void MapWaypoints::_populateLists(string file) {
  ifstream in_map_(file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    addWaypoint(x, y, s, d_x, d_y);
  }
}

void MapWaypoints::addWaypoint(double x, double y, double s, double d_x,
                               double d_y) {
  _map_x.push_back(x);
  _map_y.push_back(y);
  _map_s.push_back(s);
  _map_dx.push_back(d_x);
  _map_dy.push_back(d_y);
}

int MapWaypoints::getNextWaypoint(double x, double y, double theta) {
  return NextWaypoint(x, y, theta, _map_x, _map_y);
}

int MapWaypoints::getClosestWaypoint(double x, double y) {
  return ClosestWaypoint(x, y, _map_x, _map_y);
}

vector<double> MapWaypoints::frenetToXY(double x, double y) {
  return getXY(x, y, _map_s, _map_x, _map_y);
}

vector<double> MapWaypoints::xyToFrenet(double x, double y, double theta) {
  return getFrenet(x, y, theta, _map_x, _map_y);
}
