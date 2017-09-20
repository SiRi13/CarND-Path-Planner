#include "vehicle.hpp"
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <string>

double velocity_cost(Vehicle *dummy) {
  if (dummy->v + dummy->a == 0) {
    return 1;
  } else {
    return 1.0 / (dummy->v + dummy->a);
  }
}

double off_road_cost(Vehicle *dummy) {
  return 0 <= dummy->lane && dummy->lane < dummy->lanes_available ? 0 : 1;
}

double change_lane_cost(Vehicle *dummy) {
  double cost = 0.0;
  double lane = dummy->lane;
  if (dummy->state.compare("PLCL") == 0) {
    lane += 0.5;
  } else if (dummy->state.compare("PLCR") == 0) {
    lane -= 0.5;
  }
  cost = 1;
  cost -= exp(-abs(lane - dummy->goal_lane) / (dummy->goal_s - dummy->s));

  return cost;
}

Vehicle::Vehicle(int lane, int s, int v, int a) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = "CS";
  this->max_acceleration = -1;
}

Vehicle::~Vehicle(){};

void Vehicle::update_state(map<int, vector<vector<int>>> predictions) {
  /*
      Updates the "state" of the vehicle by assigning one of the
      following values to 'self.state':

      "KL" - Keep Lane
       - The vehicle will attempt to drive its target speed, unless there is
         traffic in front of it, in which case it will slow down.

      "LCL" or "LCR" - Lane Change Left / Right
       - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
         behavior for the "KL" state in the new lane.

      "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
       - The vehicle will find the nearest vehicle in the adjacent lane which is
         BEHIND itself and will adjust speed to try to get behind that vehicle.

      INPUTS
      - predictions
      A dictionary. The keys are ids of other vehicles and the values are arrays
      where each entry corresponds to the vehicle's predicted location at the
      corresponding timestep. The FIRST element in the array gives the vehicle's
      current position. Example (showing a car with id 3 moving at 2 m/s):

      {
        3 : [
          {"s" : 4, "lane": 0},
          {"s" : 6, "lane": 0},
          {"s" : 8, "lane": 0},
          {"s" : 10, "lane": 0},
        ]
      }

      */
  _cost_functions.push_back(&velocity_cost);
  _weights.push_back(1.0);

  _cost_functions.push_back(&off_road_cost);
  _weights.push_back(999.0);

  _cost_functions.push_back(&change_lane_cost);
  _weights.push_back(4.0);

  map<string, double> costs;
  vector<string> possible_states = _get_possible_states();
  for (auto state : possible_states) {
    double state_cost = 0.0;

    Vehicle *dummy = _create_dummy(&predictions, state);

    for (unsigned c_id = 0; c_id < _cost_functions.size(); ++c_id) {
      auto c_func = _cost_functions.at(c_id);

      double single_cost = c_func(dummy);
      double cost_weight = _weights.at(c_id);
      state_cost += cost_weight * single_cost;
    }

    costs.insert(pair<string, double>(state, state_cost));
  }

  string best_successor_state = "KL";
  double min_state_cost = numeric_limits<double>::max();
  for (auto cost_per_state : costs) {
    if (min_state_cost > cost_per_state.second) {
      best_successor_state = cost_per_state.first;
      min_state_cost = cost_per_state.second;
    }
  }

  this->state = best_successor_state;
}

vector<string> Vehicle::_get_possible_states() {
  vector<string> successor_states;

  if (this->state.compare("KL") == 0) {
    successor_states.push_back("KL");
    successor_states.push_back("PLCL");
    successor_states.push_back("PLCR");
  } else if (this->state.compare("LCL") == 0) {
    successor_states.push_back("KL");
  } else if (this->state.compare("LCR") == 0) {
    successor_states.push_back("KL");
  } else if (this->state.compare("PLCL") == 0) {
    successor_states.push_back("KL");
    successor_states.push_back("PLCL");
    successor_states.push_back("LCL");
  } else if (this->state.compare("PLCR") == 0) {
    successor_states.push_back("KL");
    successor_states.push_back("PLCR");
    successor_states.push_back("LCR");
  }

  return successor_states;
}

Vehicle *Vehicle::_create_dummy(map<int, vector<vector<int>>> *prediction,
                                string new_state) {
  Vehicle *dummy = new Vehicle(this->lane, this->s, this->v, this->a);
  dummy->configure({this->target_speed, this->lanes_available, this->goal_s,
                    this->goal_lane, this->max_acceleration});
  dummy->state = new_state;
  dummy->realize_state(*prediction);

  return dummy;
}

void Vehicle::configure(vector<int> road_data) {
  this->target_speed = road_data[0];
  this->lanes_available = road_data[1];
  this->goal_s = road_data[2];
  this->goal_lane = road_data[3];
  this->max_acceleration = road_data[4];
}

string Vehicle::display() {
  ostringstream oss;
  oss << "s: " << s << endl;
  oss << "lane: " << lane << endl;
  oss << "v: " << v << endl;
  oss << "a: " << a << endl;
  return oss.str();
}

void Vehicle::increment(int dt = 1) {
  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {
  /*
      Predicts state of vehicle in t seconds (assuming constant
     acceleration)
      */
  int s = this->s + this->v * t + this->a * t * t / 2;
  int v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {
  /*
   Simple collision detection.
  */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<int>>> predictions) {
  /*
Given a state, realize it by adjusting acceleration and lane.
Note - lane changes happen instantaneously.
*/
  string state = this->state;
  if (state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }
}
void Vehicle::realize_constant_speed() { this->a = 0; }

int Vehicle::_max_accel_for_lane(map<int, vector<vector<int>>> predictions,
                                 int lane, int s) {

  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int>>>::iterator it = predictions.begin();
  vector<vector<vector<int>>> in_front;
  while (it != predictions.end()) {
    vector<vector<int>> v = it->second;

    if ((v[0][0] == lane) && (v[0][1] > s)) {
      in_front.push_back(v);
    }
    it++;
  }

  if (in_front.size() > 0) {
    int min_s = 1000;
    vector<vector<int>> leading = {};
    for (unsigned i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - s) < min_s) {
        min_s = (in_front[i][0][1] - s);
        leading = in_front[i];
      }
    }

    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }

  return max_acc;
}

void Vehicle::realize_keep_lane(map<int, vector<vector<int>>> predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<int>>> predictions,
                                  string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(
    map<int, vector<vector<int>>> predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  int lane = this->lane + delta;

  map<int, vector<vector<int>>>::iterator it = predictions.begin();
  vector<vector<vector<int>>> at_behind;
  while (it != predictions.end()) {
    vector<vector<int>> v = it->second;

    if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }
    it++;
  }
  if (at_behind.size() > 0) {

    int max_s = -1000;
    vector<vector<int>> nearest_behind = {};
    for (unsigned i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if (delta_v != 0) {

      int time = -2 * delta_s / delta_v;
      int a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      int my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }
  }
}

vector<vector<int>> Vehicle::generate_predictions(int horizon) {
  vector<vector<int>> predictions;
  for (int i = 0; i < horizon; i++) {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;
}
