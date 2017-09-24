#include "behaviour.planner.hpp"

BehaviourPlanner::BehaviourPlanner() {
  /* EMPTY */
  _peerCarsPerLane[CENTER_LANE][-1] = PeerCar();
  _peerCarsPerLane[LEFT_LANE][-1] = PeerCar();
  _peerCarsPerLane[RIGHT_LANE][-1] = PeerCar();
}

int BehaviourPlanner::_determine_lane(const PeerCar &car) const {
  if (car._d < 4) {
    // lane 0 (lefthand)
    return LEFT_LANE;
  } else if (car._d >= 4 && car._d < 8) {
    // lane 1 (center)
    return CENTER_LANE;
  } else if (car._d >= 8) {
    // lane 2 (righthand)
    return RIGHT_LANE;
  } else {
    // ERROR
    return 3;
  }
}

void BehaviourPlanner::_lane_change_left() {
  // printf("_lane_change_left\n");
  vector<vector<double>> cars_ahead_left = _peerCarsAhead[LEFT_LANE];
  if (cars_ahead_left.size() > 0) {
    // sort by peerCar._s
    std::sort(cars_ahead_left.begin(), cars_ahead_left.end(),
              [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
    printf("car[%i] ahead_left @ %f in %f with d: %f\n",
           (int)cars_ahead_left[0][0], _lane_v * MPH_TO_MS,
           cars_ahead_left[0][1],
           _peerCarsPerLane[LEFT_LANE][cars_ahead_left[0][0]]._d);
    if (cars_ahead_left[0][1] <= MIN_SAFETY_DIST) {
      // car in same lane and too close
      _lane_v = cars_ahead_left[0][2];
      _next_state = "FC";
      vector<vector<double>> cars_ahead_center = _peerCarsAhead[CENTER_LANE];
      printf("cars_ahead_center: %i\n", (int)cars_ahead_center.size());
      if (cars_ahead_center.size() > 0) {
        std::sort(
            cars_ahead_center.begin(), cars_ahead_center.end(),
            [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
        printf("car[%i] ahead_center @ %f in %f\n",
               (int)cars_ahead_center[0][0],
               cars_ahead_center[0][2] * MPH_TO_MS, cars_ahead_center[0][1]);
        if (cars_ahead_center[0][1] > LANE_CHANGE_DIST_AHEAD) {
          // car in center lane but far away
          vector<vector<double>> cars_behind_center =
              _peerCarsBehind[CENTER_LANE];

          if (cars_behind_center.size() > 0) {
            std::sort(
                cars_behind_center.begin(), cars_behind_center.end(),
                [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
            printf("car[%i] behind center @ %f in %f\n",
                   (int)cars_behind_center[0][0],
                   cars_behind_center[0][2] * MPH_TO_MS,
                   cars_behind_center[0][1]);
            if (abs(cars_behind_center[0][1]) > LANE_CHANGE_DIST_BEHIND) {
              _lane_idx = CENTER_LANE;
              _next_state = "CLL";
              _lane_v = MAX_VELOCITY;
            }
          } else {
            _lane_idx = CENTER_LANE;
            _next_state = "CLL";
            _lane_v = MAX_VELOCITY;
          }
        }
      } else {
        _lane_v = MAX_VELOCITY;
        _lane_idx = CENTER_LANE;
        _next_state = "CLL";
      }
    }
  } else {
    _lane_v = MAX_VELOCITY;
    _next_state = "KL";
  }
}

void BehaviourPlanner::_lane_change_center() {
  // printf("_lane_change_center\n");
  vector<vector<double>> cars_ahead_center = _peerCarsAhead[CENTER_LANE];
  // sort by peerCar._s
  std::sort(cars_ahead_center.begin(), cars_ahead_center.end(),
            [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
  if (cars_ahead_center.size() > 0 &&
      cars_ahead_center[0][1] <= MIN_SAFETY_DIST) {
    // cars ahead and too close
    _lane_v = cars_ahead_center[0][2];
    _next_state = "FC";
    printf("car[%i] ahead @ %f in %f\n", (int)cars_ahead_center[0][0],
           (_lane_v * MPH_TO_MS), cars_ahead_center[0][1]);
    vector<vector<double>> cars_ahead_left = _peerCarsAhead[LEFT_LANE];
    vector<vector<double>> cars_ahead_right = _peerCarsAhead[RIGHT_LANE];
    if (cars_ahead_left.size() < 1) {
      printf("no cars cars_ahead_left\n");
      // no cars on left lane
      // => switch there right away
      _lane_idx = LEFT_LANE;
      _next_state = "CLL";
      _lane_v = MAX_VELOCITY;
      return;
    } else if (cars_ahead_right.size() < 1) {
      printf("no cars cars_ahead_right\n");
      // no cars on right lane
      // => switch there right away
      _lane_idx = RIGHT_LANE;
      _next_state = "CLR";
      _lane_v = MAX_VELOCITY;
      return;
    }
    vector<vector<double>> cars_behind_left = _peerCarsBehind[LEFT_LANE];
    vector<vector<double>> cars_behind_right = _peerCarsBehind[RIGHT_LANE];
    std::sort(cars_ahead_left.begin(), cars_ahead_left.end(),
              [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
    std::sort(cars_ahead_right.begin(), cars_ahead_right.end(),
              [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
    std::sort(cars_behind_left.begin(), cars_behind_left.end(),
              [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
    std::sort(cars_behind_right.begin(), cars_behind_right.end(),
              [](vector<double> a, vector<double> b) { return a[1] < b[1]; });

    if ((cars_behind_left.size() < 1 && cars_behind_right.size() < 1) ||
        (abs(cars_behind_left[0][1]) > LANE_CHANGE_DIST_BEHIND &&
         abs(cars_ahead_right[0][1]) > LANE_CHANGE_DIST_BEHIND)) {
      // no cars from behind on either lane
      printf("no car from behind on either lane!\n");
      _lane_v = MAX_VELOCITY;
      if ((cars_ahead_right[0][1] > LANE_CHANGE_DIST_AHEAD &&
           cars_ahead_left[0][1] > LANE_CHANGE_DIST_AHEAD &&
           cars_ahead_left.size() < cars_ahead_right.size()) ||
          cars_ahead_left[0][1] > LANE_CHANGE_DIST_AHEAD) {
        _lane_idx = LEFT_LANE;
        _next_state = "CLL";
        _lane_v = MAX_VELOCITY; // cars_ahead_left[0][2];
        printf("car[%i] ahead left CLL @ %f in %f\n",
               (int)cars_ahead_left[0][0], _lane_v, cars_ahead_left[0][1]);
      } else if (cars_ahead_right[0][1] > LANE_CHANGE_DIST_AHEAD) {
        _lane_idx = RIGHT_LANE;
        _next_state = "CLR";
        _lane_v = MAX_VELOCITY; // cars_ahead_right[0][2];
        printf("car[%i] ahead right CLR @ %f in %f\n",
               (int)cars_ahead_right[0][0], _lane_v, cars_ahead_right[0][1]);
      }
    } else if ((cars_behind_left.size() < 1 ||
                abs(cars_behind_left[0][1]) > LANE_CHANGE_DIST_BEHIND) &&
               cars_ahead_left[0][1] > LANE_CHANGE_DIST_AHEAD) {
      printf("going to left lane\n");
      _lane_idx = LEFT_LANE;
      _next_state = "CLL";
      _lane_v = MAX_VELOCITY; // cars_ahead_left[0][2];
      printf("car[%i] ahead and behind CLL @ %f in %f\n",
             (int)cars_ahead_left[0][0], _lane_v * MPH_TO_MS,
             cars_ahead_left[0][1]);
    } else if ((cars_behind_right.size() < 1 ||
                abs(cars_behind_right[0][1]) > LANE_CHANGE_DIST_BEHIND) &&
               cars_ahead_right[0][1] > LANE_CHANGE_DIST_AHEAD) {
      printf("going to right lane\n");
      _lane_idx = RIGHT_LANE;
      _next_state = "CLR";
      _lane_v = MAX_VELOCITY; // cars_ahead_right[0][2];
      printf("car[%i] ahead and behind CLR @ %f in %f\n",
             (int)cars_ahead_right[0][0], _lane_v * MPH_TO_MS,
             cars_ahead_right[0][1]);
    }
  } else {
    _lane_v = MAX_VELOCITY;
    _next_state = "KL";
  }
}

void BehaviourPlanner::_lane_change_right() {
  // printf("_lane_change_right\n");
  vector<vector<double>> cars_ahead_right = _peerCarsAhead[RIGHT_LANE];
  if (cars_ahead_right.size() > 0) {
    std::sort(cars_ahead_right.begin(), cars_ahead_right.end(),
              [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
    printf("car[%i] ahead @ %f in %f with d: %f\n", (int)cars_ahead_right[0][0],
           cars_ahead_right[0][2] * MPH_TO_MS, cars_ahead_right[0][1],
           _peerCarsPerLane[LEFT_LANE][cars_ahead_right[0][0]]._d);
    if (cars_ahead_right[0][1] <= MIN_SAFETY_DIST) {
      _lane_v = cars_ahead_right[0][2];
      _next_state = "FC";
      printf("_peerCarsPerLane: %i\n", (int)_peerCarsAhead[CENTER_LANE].size());
      vector<vector<double>> cars_ahead_center = _peerCarsAhead[CENTER_LANE];
      if (cars_ahead_center.size() > 0) {
        std::sort(
            cars_ahead_center.begin(), cars_ahead_center.end(),
            [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
        printf("car[%i] ahead center @ %f in %f\n",
               (int)cars_ahead_center[0][0],
               cars_ahead_center[0][2] * MPH_TO_MS, cars_ahead_center[0][1]);
        if (cars_ahead_center[0][1] > LANE_CHANGE_DIST_AHEAD) {
          vector<vector<double>> cars_behind_center =
              _peerCarsBehind[CENTER_LANE];
          if (cars_behind_center.size() > 0) {
            std::sort(
                cars_behind_center.begin(), cars_behind_center.end(),
                [](vector<double> a, vector<double> b) { return a[1] < b[1]; });
            printf("car[%i] behind center @ %f in %f\n",
                   (int)cars_behind_center[0][0],
                   cars_behind_center[0][2] * MPH_TO_MS,
                   cars_behind_center[0][1]);
            if (abs(cars_behind_center[0][1]) > LANE_CHANGE_DIST_BEHIND) {
              _next_state = "CLL";
              _lane_idx = CENTER_LANE;
              _lane_v = cars_ahead_center[0][2];
            }
          } else {
            _next_state = "CLL";
            _lane_idx = CENTER_LANE;
            _lane_v = cars_ahead_center[0][2];
          }
        }
      }
    }
  } else {
    _lane_v = MAX_VELOCITY;
    _next_state = "KL";
  }
}

void BehaviourPlanner::set_sensor_data(vector<vector<double>> &sensor_data,
                                       const Path &prevPath) {
  unsigned i = 0;
  _previousPath = prevPath;
  for (i = 0; i < 6; ++i) {
    _peerCarsAhead[i].clear();
    _peerCarsBehind[i].clear();
  }
  for (i = 0; i < sensor_data.size(); ++i) {
    PeerCar pCar(sensor_data[i][SENSOR_FUSION_CAR_ID],
                 sensor_data[i][SENSOR_FUSION_CAR_X],
                 sensor_data[i][SENSOR_FUSION_CAR_Y],
                 sensor_data[i][SENSOR_FUSION_CAR_VX],
                 sensor_data[i][SENSOR_FUSION_CAR_VY],
                 sensor_data[i][SENSOR_FUSION_CAR_S],
                 sensor_data[i][SENSOR_FUSION_CAR_D]);
    int laneIdx = _determine_lane(pCar);
    if (laneIdx == 3) {
      printf("\terror car: %i, %f, %f, %f\n", (int)pCar._id, pCar._s, pCar._d,
             pCar.get_velocity());
    }
    _peerCarsPerLane[laneIdx][pCar._id] = pCar;

    double future_s = pCar.get_future_s(_previousPath.get_path_size());
    // printf("future_s = %f\n", future_s);
    if (future_s <= _egoCar->_s) {
      // behind
      _peerCarsBehind[laneIdx].push_back(
          {pCar._id, (future_s - _egoCar->_s), pCar.get_velocity()});
      // printf("carBehind: %i, %f, %f\n", (int)pCar._id, (future_s -
      // _egoCar->_s),
      // pCar.get_velocity());
    } else {
      // ahead
      _peerCarsAhead[laneIdx].push_back(
          {pCar._id, (future_s - _egoCar->_s), pCar.get_velocity()});
      if (laneIdx == RIGHT_LANE) {
        printf("carAheadRight: %i, %f, %f, %f\n", (int)pCar._id,
               (future_s - _egoCar->_s), pCar._d, pCar.get_velocity());
      }
    }
  }
  _next_state = "KL";
  _lane_v = MAX_VELOCITY;
  printf("_peerCarsPerLane: %i|%i|%i\t_peerCarsAhead: "
         "%i|%i|%i\t_peerCarsBehind: %i|%i|%i\n",
         (int)_peerCarsPerLane[LEFT_LANE].size(),
         (int)_peerCarsPerLane[CENTER_LANE].size(),
         (int)_peerCarsPerLane[RIGHT_LANE].size(),
         (int)_peerCarsAhead[LEFT_LANE].size(),
         (int)_peerCarsAhead[CENTER_LANE].size(),
         (int)_peerCarsAhead[RIGHT_LANE].size(),
         (int)_peerCarsBehind[LEFT_LANE].size(),
         (int)_peerCarsBehind[CENTER_LANE].size(),
         (int)_peerCarsBehind[RIGHT_LANE].size());
  switch (_lane_idx) {
  case LEFT_LANE:
    // printf("LEFT_LANE \n");
    _lane_change_left();
    break;
  case CENTER_LANE:
    // printf("CENTER_LANE \n");
    _lane_change_center();
    break;
  case RIGHT_LANE:
    printf("RIGHT_LANE \n");
    // _lane_change_right();
    break;
  }
}

/*
 * returns { laneIdx, laneVelocity, state }
 */
vector<double> BehaviourPlanner::get_next_lane() const {
  // printf("laneidx %i\tlane_v: %f\n", _lane_idx, (_lane_v * MPH_TO_MS));
  return {double(_lane_idx), (_lane_v * MPH_TO_MS)};
}

string BehaviourPlanner::get_next_state() const {
  // printf("next_state: %s\n", _next_state.c_str());
  return _next_state;
}

void BehaviourPlanner::set_ego_car(EgoCar &ego) { this->_egoCar = &ego; }
