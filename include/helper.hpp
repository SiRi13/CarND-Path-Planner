#ifndef HELPER_HPP_
#define HELPER_HPP_

#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>
#include "json.hpp"

// time interval of simulator in ms
#define DELTA_T 0.02

#define MAX_VELOCITY 49.5
#define DELTA_VELOCITY 0.224

#define MPH_TO_MS 2.24
#define MIN_SAFETY_DIST 40

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
#endif  // HELPER_HPP_
