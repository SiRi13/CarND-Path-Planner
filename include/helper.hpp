#ifndef HELPER_HPP_
#define HELPER_HPP_

#include "json.hpp"
#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>

// time interval of simulator in ms
#define DELTA_T 0.02

#define MAX_VELOCITY 49.5
#define DELTA_VELOCITY 0.224

#define MPH_TO_MS 2.24
#define MIN_SAFETY_DIST 30
#define LANE_CHANGE_DIST_AHEAD 40
#define LANE_CHANGE_DIST_BEHIND 15

#define SENSOR_FUSION_CAR_ID 0
#define SENSOR_FUSION_CAR_X 1
#define SENSOR_FUSION_CAR_Y 2
#define SENSOR_FUSION_CAR_VX 3
#define SENSOR_FUSION_CAR_VY 4
#define SENSOR_FUSION_CAR_S 5
#define SENSOR_FUSION_CAR_D 6

#define WAYPOINTS_SPACING 30

#define LEFT_LANE 0
#define CENTER_LANE 1
#define RIGHT_LANE 2

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);

std::string hasData(std::string s);

struct Path {
public:
  Path() { Path({0.0}, {0.0}, 0.0, 0.0); }
  Path(vector<double> x_vals, vector<double> y_vals, double end_s, double end_d)
      : _path_x(x_vals), _path_y(y_vals), _end_path_s(end_s),
        _end_path_d(end_d) {}
  unsigned int get_path_size() { return _path_x.size(); }

  vector<double> _path_x, _path_y;
  double _end_path_s, _end_path_d;

  void print() {
    printf("x_vals: %f\ty_vals: %f\tend_s=%f\tend_d=%f\n", _path_x, _path_y,
           _end_path_s, _end_path_s);
  }
};

struct EgoCar {
  EgoCar() { EgoCar(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
  EgoCar(double x, double y, double s, double d, double yaw, double v)
      : _x(x), _y(y), _s(s), _d(d), _yaw(yaw), _v(v) {}

  double _x, _y, _s, _d, _yaw, _v;
  Path _path;

  void print() {
    printf("x=%f\ty=%f\ts=%f\td=%f\tyaw=%f\tv=%f\n", _x, _y, _s, _d, _yaw, _v);
  }
};

struct PeerCar {
  /*data format for each car is: [ id, x, y, vx, vy, s, d]*/
  PeerCar() { PeerCar(-1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
  PeerCar(double id, double x, double y, double vx, double vy, double s,
          double d)
      : _id(id), _x(x), _y(y), _vx(vx), _vy(vy), _s(s), _d(d) {}

  double _id, _x, _y, _vx, _vy, _s, _d;
  double _other_car_v = -1.0;
  double _future_s;
  double get_velocity() {
    if (_other_car_v < 0.0) {
      // calculate velocity magnitude of car from vx and vy
      _other_car_v = sqrt((_vx * _vx) + (_vy * _vy));
    }
    return _other_car_v;
  }
  double get_future_s(const unsigned int prev_path_size) {
    _future_s = _s;
    if (prev_path_size > 0) {
      _future_s += ((double)prev_path_size * DELTA_T * get_velocity());
    }
    return _future_s;
  }
};

#endif // HELPER_HPP_
