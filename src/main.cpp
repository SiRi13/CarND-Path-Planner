#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helper.hpp"
#include "json.hpp"
#include "mapwaypoints.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file = "../data/highway_map.csv";
  MapWaypoints waypoints(map_file);

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  // max velocity in mph
  double ref_v = DELTA_VELOCITY;
  // zero-based lane index from left to right
  int lane = 1;

  h.onMessage([&waypoints, &max_s, &ref_v, &lane](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          // double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          // double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          // double end_path_d = j[1]["end_path_d"];

          unsigned prev_path_size = previous_path_x.size();

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds

          if (prev_path_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          double lane_v = MAX_VELOCITY;

          // check sensor data of each detected vehicle
          for (unsigned i = 0; i < sensor_fusion.size(); ++i) {
            // get lane of car
            float d = sensor_fusion[i][SENSOR_FUSION_CAR_D];
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
              // calculate velocity magnitude of car from vx and vy
              double vx = sensor_fusion[i][SENSOR_FUSION_CAR_VX];
              double vy = sensor_fusion[i][SENSOR_FUSION_CAR_VY];
              double other_car_v = sqrt((vx * vx) + (vy * vy));

              // calculate s for other car when previous path is driven
              double other_car_s = sensor_fusion[i][SENSOR_FUSION_CAR_S];
              other_car_s += ((double)prev_path_size * 0.02 * other_car_v);

              if (other_car_s > car_s) {
                double dist = other_car_s - car_s;
                if (dist < MIN_SAFETY_DIST) {
                  too_close = true;
                  if (lane_v > other_car_v) {
                    lane_v = other_car_v;
                  }
                } else if (dist < MIN_SAFETY_DIST * 1.1) {
                  lane_v = other_car_v;
                }
              }
            }
          }

          // anchor points which are widely spread and evenly spaced
          std::vector<double> x_anchor;
          std::vector<double> y_anchor;

          // init reference point with current position and direction
          double ref_x = car_x, ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double ref_x_prev, ref_y_prev;

          // enough previous points?
          if (prev_path_size > 2) {
            // use last previous point for reference
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];

            // point before reference
            ref_x_prev = previous_path_x[prev_path_size - 2];
            ref_y_prev = previous_path_y[prev_path_size - 2];
            ref_yaw = atan2((ref_y - ref_y_prev), (ref_x - ref_x_prev));
          } else {
            // interpolate previous point
            ref_x_prev = car_x - cos(car_yaw);
            ref_y_prev = car_y - sin(car_yaw);
          }
          // push points to anchor
          x_anchor.push_back(ref_x_prev);
          x_anchor.push_back(ref_x);
          y_anchor.push_back(ref_y_prev);
          y_anchor.push_back(ref_y);

          // create and push three new anchor points
          std::vector<double> anchor;
          for (unsigned i = 1; i <= 3; ++i) {
            double anchor_s = car_s + (i * 30);
            double anchor_d = 2 + 4 * lane;
            anchor = waypoints.frenetToXY(anchor_s, anchor_d);
            x_anchor.push_back(anchor[0]);
            y_anchor.push_back(anchor[1]);
          }

          // transformation to local car coordinates
          for (unsigned i = 0; i < x_anchor.size(); ++i) {
            double shift_x = x_anchor[i] - ref_x;
            double shift_y = y_anchor[i] - ref_y;

            x_anchor[i] =
                shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            y_anchor[i] =
                shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create spline
          tk::spline anchor_spline;
          // set anchor points
          anchor_spline.set_points(x_anchor, y_anchor);

          // define next car position points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (unsigned i = 0; i < prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0, target_y = anchor_spline(target_x);
          double target_dist =
              sqrt((target_x * target_x) + (target_y + target_y));
          double x_next_pos = 0.0;

          for (unsigned i = 0; i < 50 - prev_path_size; ++i) {
            if ((too_close && ref_v > lane_v) ||
                (!too_close && lane_v < MAX_VELOCITY)) {
              ref_v -= DELTA_VELOCITY;
            } else if (too_close) {
              printf("too_close: %f = %f\n", ref_v, lane_v);
              ref_v = lane_v * 1.25;
            } else if (ref_v < MAX_VELOCITY) {
              ref_v += DELTA_VELOCITY;
            }

            double n_pos_points = target_dist / (0.02 * ref_v / MPH_TO_MS);
            double x = x_next_pos + target_x / n_pos_points;
            double y = anchor_spline(x);

            x_next_pos = x;
            double x_ref = x, y_ref = y;
            x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x += ref_x;
            y += ref_y;

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
