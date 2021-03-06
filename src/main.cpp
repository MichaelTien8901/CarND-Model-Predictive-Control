#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // ways points
          size_t i;
          vector<double> ptsx = j[1]["ptsx"]; // ptsx, ptsy in global coordinate
          vector<double> ptsy = j[1]["ptsy"]; // 
          double px = j[1]["x"]; 
          double py = j[1]["y"];  
          double psi = j[1]["psi"]; // radian
          double v = j[1]["speed"]; // mph
          double v_msec = v * 0.44704; // convert to m/sec
          // get steering angle
          double delta = j[1]["steering_angle"];
          delta *= -0.4363323; // simulator steering angle in opposite direction

          const double latency = 0.1; // 100ms latency of actuator
          const double Lf = 2.67;
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt          
          px = px + v_msec * cos(psi) * latency;
          py = py + v_msec * sin(psi) * latency;
          psi = psi + v_msec*delta/Lf*latency;
          double a = j[1]["throttle"];
          v = v + a * latency;

          Eigen::VectorXd eptsx(ptsx.size()); // transformed waypoints
          Eigen::VectorXd eptsy(ptsy.size());

          // convert waypoints coordinate into vehicle coordinate
          for (i = 0; i < ptsx.size(); i ++) {
              double sx = ptsx[i] - px;
              double sy = ptsy[i] - py;
              eptsx[i] = sx * cos(psi) + sy * sin(psi);
              eptsy[i] = - sx * sin(psi) + sy * cos(psi);
          }
          // fit into poly
          auto coeffs = polyfit(eptsx, eptsy, 3);
          
          // cross track error, at x = 0
          double cte = polyeval(coeffs, 0) - py;
          // epsi
          // derivative of coeffs[0] + coeffs[1] * x + coeffs[2] * x ^2 + coeffs[3] * x ^3
          // -> coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * x ^ 2
          // At vehicle coordinate, x = 0, only leave coeffs[1]
          double epsi = - atan(coeffs[1]); // psi - atan(), where psi = 0

          Eigen::VectorXd state(6);
          // vehicle coordinate x, y = 0, psi = 0
          // state << px, py, psi, v_msec, cte, epsi;
          state << 0, 0, 0, v_msec, cte, epsi;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          auto vars = mpc.Solve(state, coeffs);
          // return size = 2 + 2(N-1) = 2N
          size_t N = vars.size() / 2;
          double steer_value = vars[0] / 0.4363323; // normalize to be [-1, 1]
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value; // simulator steering angle direction is opposite
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for ( i = 0; i < (N-1); i ++ ) {
              mpc_x_vals.push_back( vars[2 + i * 2    ]);
              mpc_y_vals.push_back( vars[2 + i * 2 + 1]);
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          // for ( i = 0; i < (N-1); i ++) {
          //   std::cout << "mpc_x = " << mpc_x_vals[i] << std::endl;
          //   std::cout << "mpc_y = " << mpc_y_vals[i] << std::endl;

          // }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // ** Use the transformed waypoints 
          // next_x_vals.resize(eptsx.size());
          // Eigen::VectorXd::Map(&next_x_vals[0], eptsx.size()) = eptsx;

          // next_y_vals.resize(eptsy.size());
          // Eigen::VectorXd::Map(&next_y_vals[0], eptsy.size()) = eptsy;

          // get from transform waypoints 
          vector<double> next_x_vals(eptsx.data(), eptsx.data() + eptsx.size()); 
          vector<double> next_y_vals(eptsy.data(), eptsy.data() + eptsy.size());

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          // for ( i = 0; i < next_x_vals.size(); i ++) {
          //   std::cout << "next_x = " << next_x_vals[i] << std::endl;
          //   std::cout << "next_y = " << next_y_vals[i] << std::endl;            
          // }

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
