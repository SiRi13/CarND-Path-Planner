#include "path.planner.hpp"

void PathPlanner::set_data(string data) {
  // cout << "parse data" << endl;
  auto j = json::parse(data);
  string event = j[0].get<string>();
  if (event == "telemetry") {
    // cout << "telemtry" << endl;
    _egoCar = EgoCar(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"],
                     j[1]["speed"]);

    // cout << "_previousPath" << endl;
    _previousPath = Path(j[1]["previous_path_x"], j[1]["previous_path_y"],
                         j[1]["end_path_s"], j[1]["end_path_d"]);
    //  _previousPath.print();

    if (_previousPath.get_path_size() > 0) {
      _egoCar._s = _previousPath._end_path_s;
    }

    // cout << "sensor_fusion" << endl;
    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
    _sensor_fusion = sensor_fusion;
  }
}

void PathPlanner::createPath() {
  unsigned i = 0;
  _behaviourPlanner.set_ego_car(_egoCar);
  _behaviourPlanner.set_sensor_data(_sensor_fusion, _previousPath);
  string next_state = _behaviourPlanner.get_next_state();
  vector<double> next_lane = _behaviourPlanner.get_next_lane();
  int lane = next_lane[0];
  double lane_v = next_lane[1];

  // anchor points which are widely spread and evenly spaced
  std::vector<double> x_anchor;
  std::vector<double> y_anchor;

  // init reference point with current position and direction
  double ref_x = _egoCar._x, ref_y = _egoCar._y;
  double ref_yaw = deg2rad(_egoCar._yaw);
  double ref_x_prev, ref_y_prev;

  // enough previous points?
  if (_previousPath.get_path_size() > 2) {
    // use last previous point for reference
    ref_x = _previousPath._path_x[_previousPath.get_path_size() - 1];
    ref_y = _previousPath._path_y[_previousPath.get_path_size() - 1];

    // point before reference
    ref_x_prev = _previousPath._path_x[_previousPath.get_path_size() - 2];
    ref_y_prev = _previousPath._path_y[_previousPath.get_path_size() - 2];
    ref_yaw = atan2((ref_y - ref_y_prev), (ref_x - ref_x_prev));
  } else {
    // interpolate previous point
    ref_x_prev = _egoCar._x - cos(_egoCar._yaw);
    ref_y_prev = _egoCar._y - sin(_egoCar._yaw);
  }

  // push points to anchor
  x_anchor.push_back(ref_x_prev);
  x_anchor.push_back(ref_x);
  y_anchor.push_back(ref_y_prev);
  y_anchor.push_back(ref_y);

  // create and push three new anchor points
  std::vector<double> anchor;
  for (unsigned i = 1; i <= 3; ++i) {
    double anchor_s = _egoCar._s + (i * 30);
    double anchor_d = 2 + 4 * lane;
    anchor = _waypoints.frenetToXY(anchor_s, anchor_d);
    x_anchor.push_back(anchor[0]);
    y_anchor.push_back(anchor[1]);
  }

  // transformation to local car coordinates
  for (unsigned i = 0; i < x_anchor.size(); ++i) {
    double shift_x = x_anchor[i] - ref_x;
    double shift_y = y_anchor[i] - ref_y;

    x_anchor[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    y_anchor[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // create spline
  tk::spline anchor_spline;
  // set anchor points
  anchor_spline.set_points(x_anchor, y_anchor);

  for (unsigned i = 0; i < _previousPath.get_path_size(); ++i) {
    _egoCar._path._path_x.push_back(_previousPath._path_x[i]);
    _egoCar._path._path_y.push_back(_previousPath._path_y[i]);
  }
  // _egoCar._path.print();

  double target_x = 30.0, target_y = anchor_spline(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y + target_y));
  double x_next_pos = 0.0;

  for (unsigned i = 0; i < 50 - _previousPath.get_path_size(); ++i) {
    /*if ((next_state == "KL" && _ref_v > lane_v) ||
        (next_state != "KL" && lane_v < MAX_VELOCITY)) {
      _ref_v -= DELTA_VELOCITY;
    } else if (next_state == "FC") {
      printf("too_close: %f = %f\n", _ref_v, lane_v);
      _ref_v = lane_v * 1.25;
    } else if (_ref_v < MAX_VELOCITY) {
      _ref_v += DELTA_VELOCITY;
    }
    */
    if (next_state == "FC") {
      if (_ref_v > lane_v) {
        _ref_v -= DELTA_VELOCITY;
      } else if (_ref_v <= lane_v) {
        _ref_v += DELTA_VELOCITY;
      }
    } else if (next_state == "CLL" || next_state == "CLR") {
      if (_ref_v < lane_v) {
        _ref_v += DELTA_VELOCITY;
      } else if (_ref_v > lane_v) {
        _ref_v -= DELTA_VELOCITY;
      }

    } else if (_ref_v < MAX_VELOCITY) {
      _ref_v += DELTA_VELOCITY;
    }

    if (_ref_v > MAX_VELOCITY) {
      _ref_v = MAX_VELOCITY;
    }

    double n_pos_points = target_dist / (0.02 * _ref_v / MPH_TO_MS);
    double x = x_next_pos + target_x / n_pos_points;
    double y = anchor_spline(x);

    x_next_pos = x;
    double x_ref = x, y_ref = y;
    x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x += ref_x;
    y += ref_y;

    _egoCar._path._path_x.push_back(x);
    _egoCar._path._path_y.push_back(y);
  }
}

std::array<std::vector<double>, 2> PathPlanner::get_next_vals() {
  std::array<std::vector<double>, 2> retVal;
  retVal[0] = _egoCar._path._path_x;
  retVal[1] = _egoCar._path._path_y;
  return retVal;
}
