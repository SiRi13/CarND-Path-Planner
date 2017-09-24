#ifndef BEHAVIOUR_PLANNER_HPP_
#define BEHAVIOUR_PLANNER_HPP_

#include "helper.hpp"
#include <map>
#include <string>
#include <vector>

using namespace std;

class BehaviourPlanner {
private:
  double _v_ref = DELTA_VELOCITY;
  // zero-based lane index from left to right
  int _lane_idx = CENTER_LANE;
  // The max s value before wrapping around the track back to 0
  double _max_s = 6945.554;

  string _next_state = "KL";
  double _lane_v = MAX_VELOCITY;

  /* ego car reference */
  EgoCar *_egoCar;

  /* laneIdx:{PeerCar} */
  map<int, map<int, PeerCar>> _peerCarsPerLane;
  /* laneIdx:{{carId, future_dist, car_v}} Map */
  map<int, vector<vector<double>>> _peerCarsAhead;
  /* laneIdx:{{carId, future_dist, car_v}} Map */
  map<int, vector<vector<double>>> _peerCarsBehind;

  Path _previousPath;

  int _determine_lane(const PeerCar &car) const;
  void _lane_change_left();
  void _lane_change_center();
  void _lane_change_right();

public:
  BehaviourPlanner();
  virtual ~BehaviourPlanner() {}

  void set_ego_car(EgoCar &ego);
  void set_sensor_data(vector<vector<double>> &sensor_fusion,
                       const Path &prevPath);
  /* returns { laneIndex, lane_v } */
  vector<double> get_next_lane() const;
  string get_next_state() const;
};

#endif // BEHAVIOUR_PLANNER_HPP_
