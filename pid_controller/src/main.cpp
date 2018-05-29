#include "PID.h"
#include "json.hpp"
#include <iostream>
#include <math.h>
#include <uWS/uWS.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  PID pid_throttle;
  if (argc == 7) {
    pid.Init(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    pid_throttle.Init(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  } else {
    pid.Init(0.2, 0.0041, 2.9469);
    pid_throttle.Init(0.2, 0.004, 3.0);
  }

  int step = 0; // num step from begining to control switch of throttle PID

  h.onMessage([&pid,&pid_throttle,&step](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
           * TODO: Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe
           * use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          // steer_value = pid.TotalError() + 0.3;  // I component effect check

          double throttle = 0.3;
          double base_speed = 25.0; // expected base speed to avoid slow
          // Control throttle after finishing almost all of aranging steer
          if (step >= 300) {
          // if (false && step >= 300) {  // turn off throttle control
            // Let vehicle control speed to keep low cte
            // and try to keep base speed to avoid slow
            double throttle_error = cte + (speed - base_speed) / 10;
            pid_throttle.UpdateError(throttle_error);
            throttle = pid_throttle.TotalError();
          } else {
            step++;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
  // program doesn't compile :-(
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
