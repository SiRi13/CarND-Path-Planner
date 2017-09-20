#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include <math.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

enum CostFunctionsEnum { change_lane, off_road, velocity };

class Vehicle {
 private:
  vector<string> _get_possible_states();
  Vehicle *_create_dummy(map<int, vector<vector<int>>> *predictions,
                         string new_state);
  double _calculate_cost(CostFunctionsEnum cost_func, Vehicle *dummy,
                         map<int, vector<vector<int>>> *predictions);
  double _change_lane_cost(Vehicle *dummy);
  double _velocity_cost(Vehicle *dummy);
  double _off_road_cost(Vehicle *dummy);

  vector<double> _weights;
  vector<double (*)(Vehicle *dummy)> _cost_functions;

 public:
  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
  };

  int L = 1;

  int preferred_buffer = 6;  // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
   * Constructor
   */
  Vehicle(int lane, int s, int v, int a);

  /**
   * Destructor
   */
  virtual ~Vehicle();

  void update_state(map<int, vector<vector<int>>> predictions);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector<vector<int>>> predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int, vector<vector<int>>> predictions, int lane,
                          int s);

  void realize_keep_lane(map<int, vector<vector<int>>> predictions);

  void realize_lane_change(map<int, vector<vector<int>>> predictions,
                           string direction);

  void realize_prep_lane_change(map<int, vector<vector<int>>> predictions,
                                string direction);

  vector<vector<int>> generate_predictions(int horizon);
};

#endif  // VEHICLE_HPP
