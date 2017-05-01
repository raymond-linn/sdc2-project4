#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// variables for twiddle algorithm to find best hyper parameters
int loop_count = 0;
// set to find parameters using twiddle, change this flag to run the actual program
// I could set it as argument for the program but it might be a problem for reviewer
bool is_opt_param_on = true; // false; // true;
// best error
double best_error = 1e04;
// need to initialize with some real parameters otherwise the car runs out of the track
// and just have to start up simulation again with different parameters.
// these parameters also adjusted manually. Can't use p = {0,0,0} and dp={1,1,1} as in the lesson
std::vector<double> p = {0.115, 0.001, 3.0};
std::vector<double> dp = {0.05, 0.001, 0.05};
// counter for three control p, i, d
int idx = 0;
int control_state = 0;

// define the twiddle algorithm here, pid object needs to be initialized with some parameters before
// passing pid reference to this twiddle algorithm.
// since this function will be running in the main loop, we can pass the different control state
// to update parameter by passing the control state from one another
void optimizeParameters(PID &pid) {
  std::cout << "PID Error: " << pid.TotalError() << "\t" << "Best Error: " << best_error << std::endl;

  if (control_state == 0) {
    // initialize the p and best error
    p[idx] += dp[idx];
    best_error = pid.TotalError();
    control_state = 1;
  }
  else if (control_state == 1){
    if(pid.TotalError() < best_error){
      best_error = pid.TotalError();
      p[idx] *= 1.1;
      // update to rotate three controls
      idx = (idx+1) % 3;
      p[idx] += dp[idx];
      control_state = 1;
    }
    else {
      p[idx] -= 2 * dp[idx];
      if (p[idx] < 0) {
        p[idx] = 0;
        idx = (idx +1) % 3;
      }
      control_state = 2;
    }
  }
  else { // for last
    if (pid.TotalError() < best_error) {
      best_error = pid.TotalError();
      dp[idx] *= 1.1;
      idx = (idx + 1) % 3;
      p[idx] += dp[idx];
      control_state = 1;
    }
    else {
      p[idx] += dp[idx];
      dp[idx] *= 0.9;
      idx = (idx + 1) % 3;
      p[idx] += dp[idx];
      control_state = 1;
    }
  }
  // intialize the pid with new parameter
  pid.Init(p[0], p[1], p[2]);
/*
 * from python implementation ...
  for (int i = 1; i < p.size(); i++){
    p[i] += dp[i];

    if (pid.TotalError() < best_error){
      best_error = pid.TotalError();
      dp[i] *= 1.1;
    }
    else {
      p[i] -= 2 * dp[i];

      if (pid.TotalError() < best_error) {
        best_error = pid.TotalError();
        dp[i] *= 1.1;
      }
      else {
        p[i] += dp[i];
        dp[i] *= 0.9;
      }
    }
  }
  pid.Init(p[0], p[1], p[2]);
*/
}

int main()
{
  uWS::Hub h;


  // Initialize the pid variable.
  // Initialize the PID

  PID pid;

  //double kp = 0.01, ki = 0.001, kd = 0.8;
  //pid.Init(kp, ki, kd);
  // with these parameters seem to be running ok. But did not test with fast speed
  pid.Init(0.15, 0.001, 3.0);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
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
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          double max_angle = 1.0;
          pid.UpdateError(cte);

          steer_value = pid.TotalError();
          if (steer_value > max_angle) {
            steer_value = max_angle;
          }
          if (steer_value < -max_angle) {
            steer_value = -max_angle;
          }

          // try to find out hyper parameters
          if (is_opt_param_on) {
            loop_count++;
            // just to run for 100 times
            if (loop_count > 100) {

              /*
              // reset the best error
              best_error = 1e06;
              // reset the message
              std::string msg = "42[\"reset\", {}]";
              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              */

              // twiddle algorithm that was implemented in lesson 16, lecture 14 and 15
              optimizeParameters(pid);

              //pid.OptimizeParameters(p, dp, best_error, control_state, idx);
              // print out to check the p i d parameters
              std::cout << "p i d values: " << "P is: " << p[0] << "\t" << "I is: " << p[1] << "\t" << "D is: " << p[2]
                        << std::endl;
              // set loop_count back
              loop_count = 0;
            }
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          } else {
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
