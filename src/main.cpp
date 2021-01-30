#ifdef UWS_VCPKG
  #include <uwebsockets/App.h>
  #include <string_view>
#else
  #include <uWS/uWS.h>
#endif 

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "json.hpp"
#include "config.h"
#include "behavior_planner.h"
#include "ego_car.h"
#include "helpers.h"
#include "map.h"

using nlohmann::json;
using std::string;
using std::vector;

#ifndef UWS_VCPKG
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map map("../data/highway_map.csv", cfg::kLapLength);
  BehaviorPlanner planner;

  h.onMessage([&map, &planner]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = HasData(data);

      if (s != "") {  
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          EgoCar ego_loc{
            j[1]["x"],
            j[1]["y"],
            j[1]["s"],
            j[1]["d"],
            DegToRad(j[1]["yaw"]),
            MphToMps(j[1]["speed"]),
          };

          PrevPathFromSim prev_path{
            j[1]["previous_path_x"],
            j[1]["previous_path_y"],
            j[1]["end_path_s"],
            j[1]["end_path_d"]
          };

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
            * DONE: define a path made up of (x,y) points that the car will visit
            *   sequentially every .02 seconds
            */
          planner.GetTrajectory(next_x_vals, next_y_vals, map, ego_loc, sensor_fusion, prev_path);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

#else 

int main() {

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map map("../data/highway_map.csv", cfg::kLapLength);
  BehaviorPlanner planner;

  uWS::App::WebSocketBehavior b;

	b.message = [&map, &planner] (auto* ws, std::string_view message, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    int length = int(message.length());
    const char* data = message.data();
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = HasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          EgoCar ego_loc{
            j[1]["x"],
            j[1]["y"],
            j[1]["s"],
            j[1]["d"],
            DegToRad(j[1]["yaw"]),
            MphToMps(j[1]["speed"]),
          };

          PrevPathFromSim prev_path{
            j[1]["previous_path_x"],
            j[1]["previous_path_y"],
            j[1]["end_path_s"],
            j[1]["end_path_d"]
          };

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
            * DONE: define a path made up of (x,y) points that the car will visit
            *   sequentially every .02 seconds
            */
          planner.GetTrajectory(next_x_vals, next_y_vals, map, ego_loc, sensor_fusion, prev_path);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws->send(msg, uWS::OpCode::TEXT);
        }  // end "telemetry" if
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg, uWS::OpCode::TEXT);
      }
    }  // end websocket if
  };

  b.open = [](auto* ws) {
    std::cout << "Connected!!!" << std::endl;
  };

  b.close = [](auto* ws, int /*code*/, std::string_view /*message*/) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  };

  b.maxPayloadLength = 64 * 1024 * 1024;  // experimental value found working well with VCPKG

  int port = 4567;
  struct PerSocketData {};
  uWS::App().ws<PerSocketData>("/*", std::move(b)).listen("127.0.0.1", port, [port](auto* listen_socket) {
    if (listen_socket) {
      std::cout << "Listening on port " << port << std::endl;
    }
    else {
    	std::cerr << "Failed to listen to port" << std::endl;
	    exit(-1);
    }
  }).run();
}

#endif 
