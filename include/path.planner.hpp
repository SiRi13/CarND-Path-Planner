#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include "behaviour.planner.hpp"
#include "helper.hpp"
#include "json.hpp"
#include "map.waypoints.hpp"
#include "spline.h"
#include <array>
#include <vector>

using namespace std;
// for convenience
using json = nlohmann::json;

class PathPlanner {
private:
  // Main car's localization Data
  /*
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  */
  EgoCar _egoCar;

  // Previous path data given to the Planner
  /*
  auto previous_path_x;
  auto previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;

  unsigned prev_path_size;
  */
  Path _previousPath;

  // Sensor Fusion Data, a list of all other cars on the same side of
  // the road.
  vector<vector<double>> _sensor_fusion;

  MapWaypoints _waypoints;
  BehaviourPlanner _behaviourPlanner;

  // max velocity in mph
  double _ref_v = DELTA_VELOCITY;

public:
  PathPlanner(const MapWaypoints &wp, const BehaviourPlanner &bp)
      : _waypoints(wp), _behaviourPlanner(bp) {}
  virtual ~PathPlanner() { /* EMPTY */
  }

  void set_behaviour_planner(BehaviourPlanner &bp);
  void set_data(string data);
  void createPath();

  std::array<vector<double>, 2> get_next_vals();
};

#endif // PATH_PLANNER_HPP_
