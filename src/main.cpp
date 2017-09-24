#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "behaviour.planner.hpp"
#include "helper.hpp"
#include "json.hpp"
#include "map.waypoints.hpp"
#include "path.planner.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file = "../data/highway_map.csv";
  cout << "init waypoints" << endl;
  MapWaypoints waypoints(map_file);

  cout << "init behaviourplanner" << endl;
  BehaviourPlanner behaviourPlanner;

  cout << "init pathplanner" << endl;
  PathPlanner pathPlanner(waypoints, behaviourPlanner);

  h.onMessage([&pathPlanner](uWS::WebSocket<uWS::SERVER> ws, char *data,
                             size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        // cout << "set_data" << endl;
        pathPlanner.set_data(s);
        // cout << "createPath" << endl;
        pathPlanner.createPath();

        json msgJson;
        msgJson["next_x"] = pathPlanner.get_next_vals()[0];
        msgJson["next_y"] = pathPlanner.get_next_vals()[1];

        auto msg = "42[\"control\"," + msgJson.dump() + "]";
        // cout << "msgJson: " << msg << endl;

        // this_thread::sleep_for(chrono::milliseconds(1000));
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
