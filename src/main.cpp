#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "helper_functions.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std;


constexpr double NEXT_VAL_POLY_DELTA = 2.5;
constexpr double NEXT_VAL_LENGTH     = 25;

const double Lf = 2.67;


// We add to the simulator 100ms latency
constexpr int RESPONSE_LATENCY = 100;

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

// Evaluate a polynomial tangent.
double polytang(Eigen::VectorXd coeffs, double x) {
  double result = coeffs[1];
  for (int i = 2; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return atan(result);
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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


// Use the kinematic model to get the next state
Eigen::VectorXd kinematic_model(Eigen::VectorXd state, Eigen::VectorXd actuators, Eigen::VectorXd coeffs, double dt) {
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  double delta = actuators[0];
  double a     = actuators[1];

  double f      = polyeval(coeffs, x);
  double psides = polytang(coeffs, x); 

  // Use the kinematic model to calculate the next state:
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  // v_[t+1] = v[t] + a[t] * dt
  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
  double next_x    = x + v * cos(psi) * dt;
  double next_y    = y + v * sin(psi) * dt;
  double next_psi  = psi - v * delta / Lf * dt;
  double next_v    = v + a * dt;
  double next_cte  = f - y + v * sin(epsi) * dt;
  double next_epsi = psi - psides - v * delta / Lf * dt;

  Eigen::VectorXd next_state(state.size());
  next_state << next_x, next_y, next_psi, next_v, next_cte, next_epsi;

  return next_state;
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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"]; //TODO: insert it into helper method
          double a = j[1]["throttle"];
          double steering = j[1]["steering_angle"];

          // Transform position into car coordinates system.
          double cos_delta = cos(-psi);
          double sin_delta = sin(-psi);
          for (size_t i = 0; i < ptsx.size(); ++i) {
            double offset_x = ptsx[i] - px;
            double offset_y = ptsy[i] - py;


            ptsx[i] = (offset_x * cos_delta) - (offset_y * sin_delta);
            ptsy[i] = (offset_x * sin_delta) + (offset_y * cos_delta);
          }

          // Convert position vectors into Eigen::VectorXd
          double* ptrx = &ptsx[0];
          double* ptry = &ptsy[0];

          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, ptsy.size());

          // Calculate track polynomial.
          // The polynomial rank is 3 becuase it fits most tracks shapes.
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // Calculate the errors
          double cte  = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          // Convert the velocity to Meter per seconds
          v = Helper::mph2mps(v);

          // Create state vector
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // Create actuators vector.
          Eigen::VectorXd actuators(2);
          actuators << steering, a;

          // Use kinematic model to get the car state after the response latency.
          // This step was done in order to handle the latency without letting the MPC
          // know that it exists.
          // The MPC will preform optimisation on the state that the changes in actuators values will
          // affect the vehicle state.
          state = kinematic_model(state, actuators, coeffs, Helper::millisec2sec(RESPONSE_LATENCY));

          cout << "Sending to MPC" << endl;
          // Send all results to MPC in order to get actuators.
          auto vars = mpc.Solve(state, actuators, coeffs);

          // Steering value is normalized by the max values that were set in MVC, becuase
          // in the simulator the values that the simulator is using for steering are in range 
          // of [-1, 1].
          double steer_value    = vars[0] / (Helper::deg2rad(25) * Lf);

          double throttle_value = vars[1];

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // Points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (size_t var_idx = 2; var_idx < vars.size(); var_idx += 2) {
            mpc_x_vals.push_back(vars[var_idx]);
            mpc_y_vals.push_back(vars[var_idx + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (int point = 1; point < NEXT_VAL_LENGTH; ++point) {
            next_x_vals.push_back(NEXT_VAL_POLY_DELTA * point);
            next_y_vals.push_back(polyeval(coeffs, NEXT_VAL_POLY_DELTA * point));
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(RESPONSE_LATENCY));
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
