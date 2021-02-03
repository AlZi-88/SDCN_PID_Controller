#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void resetSim(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void run(PID &pid, double cte, uWS::WebSocket<uWS::SERVER> ws){

  double steer_value;
  double throttle;

  pid.UpdateError(cte);
  steer_value = pid.TotalError();
  if (steer_value > 1.0){
    steer_value = 1.0;
  }else if (steer_value < -1.0){
    steer_value = -1.0;
  }else{
    //keep value
  }

  throttle = 0.3;
  // DEBUG
  std::cout << "CTE: " << cte << " Steering Value: " << steer_value
            << std::endl;

  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steer;
  //PID pid_speed;
  /**
   * TODO: Initialize the pid variable.
   */
   pid_steer.Init(0.27 , 0.001, 3.5);
   Twiddle twiddle(true);
   //pid_speed.Init(0.0, 0.0, 0.0);


  h.onMessage([&pid_steer, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle;
          double brake;

          if (twiddle.is_used && twiddle.sum_dp < 0.00001){
            //stop Twiddle
            twiddle.is_used = false;
          }

          if (twiddle.is_used){

            twiddle.distance += 1;

            twiddle.error_sum = cte*cte;
            twiddle.error_av = twiddle.error_sum/twiddle.distance;

            //first let the car drive a bit then start twiddle
            // after check if car is stuck or maximum distance is reached
            if (twiddle.distance > 100 && (twiddle.distance >= 50000 || std::fabs(cte) > 3.5 || speed <= 0.1)){

              if (!twiddle.init_done){
                twiddle.Init(pid_steer);
              }else{
                if (twiddle.error_av < twiddle.best_error && twiddle.distance > twiddle.best_distance){
                  twiddle.new_best_err();

                  //next parameter
                  twiddle.parameter_index += 1;
                  twiddle.parameter_index = twiddle.parameter_index%3;

                } else{
                  if (twiddle.update_factor != -2.0){
                    //decreasing update not yet done
                    twiddle.update_p(-2.0);
                    twiddle.update_pid(pid_steer);
                  } else{
                    //neither increasing nor decresing result in better update -> reest to initial values
                    twiddle.no_new_best_err();
                    twiddle.update_pid(pid_steer);

                    //next parameter
                    twiddle.parameter_index += 1;
                    twiddle.parameter_index = twiddle.parameter_index%3;
                  }

                }
              }
              if (twiddle.update_factor == 1.0){
                twiddle.update_p(1.0);
                twiddle.update_pid(pid_steer);
              }

            twiddle.distance = 0;
            twiddle.error_sum = 0;
            twiddle.error_av = 0;

            resetSim(ws);
          }

          }
          run(pid_steer, cte, ws);


      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }
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
