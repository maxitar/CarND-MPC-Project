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
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
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

// Evaluate the first derivative of a polynomial.
double polydereval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * i * pow(x, i-1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals,
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

void map2car(
    const std::vector<double>& map_x_wp, const std::vector<double>& map_y_wp, 
    double x_car, double y_car, double psi,
    std::vector<double>& car_x_wp, std::vector<double>& car_y_wp) {
  int n = map_x_wp.size();
  psi = -psi;
  double sin_p = std::sin(psi);
  double cos_p = std::cos(psi);
  for (int i = 0; i < n; ++i) {
    double x = map_x_wp[i] - x_car; 
    double y = map_y_wp[i] - y_car;
    car_x_wp[i] = cos_p*x - sin_p*y;
    car_y_wp[i] = sin_p*x + cos_p*y;
  }
}

void advanceState(Eigen::VectorXd& state, double dt, double Lf) {
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  double delta = state[6];
  double a = state[7];
  double f = cte;
  double psides = -epsi;

  state[0] = x + v*std::cos(psi)*dt;
  state[1] = y + v*std::sin(psi)*dt;
  state[2] = psi + v*delta*dt/Lf;
  state[3] = v + a*dt;
  state[4] = (f-y) + v*std::sin(epsi)*dt;
  //state[4] = (y-f) + v*std::sin(epsi)*dt;
  state[5] = psi - psides + v*delta*dt/Lf;
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
    std::string sdata = std::string(data).substr(0, length);
    //std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
    std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          //std::cout << delta << " " << a << std::endl;

          std::vector<double> x_input(ptsx.size());
          std::vector<double> y_input(ptsy.size());
          map2car(ptsx, ptsy, px, py, psi, x_input, y_input);
          int x_size = x_input.size();
          Eigen::VectorXd xs = Eigen::VectorXd::Map(x_input.data(), x_size);
          Eigen::VectorXd ys = Eigen::VectorXd::Map(y_input.data(), x_size);
          auto coeffs = polyfit(xs, ys, 3);
          //std::cout << xs << std::endl;
          //for (int i = 0; i < ptsx.size()-1; ++i) {
          //  if (x_input[i+1] < x_input[i]) {
          //    std::cout << xs << std::endl;
          //    throw;
          //  }
         // }
          double x = 0.;
          double y = 0.;
          psi = 0.;
          double cte = polyeval(coeffs, x) - y;
          //double cte = -polyeval(coeffs, x);
          double epsi = psi - polydereval(coeffs, x);
          int latency = 100; //100 //milliseconds
          Eigen::VectorXd state(8);
          state << x, y, psi, v, cte, epsi, -delta, a;
          if (latency > 0) {
            //advanceState(state, latency/1000.0, 2.67);
            advanceState(state, 0.07, 2.67);
            state[3] = v;
            //advanceState(state, 0.05, 2.67);
            //state[3] = v;
            state[4] = polyeval(coeffs,state[0]) - state[1];
            //state[4] = state[1] - polyeval(coeffs,state[0]);
            state[5] = state[2] - polydereval(coeffs,state[0]);
          }
          //std::cout << state << std::endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          std::vector<double> res = mpc.Solve(state, coeffs);
          int N = (res.size()-2)/2;
          double steer_value = res[res.size()-2];
          double throttle_value = res.back();
          std::cout << steer_value << " " << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          std::vector<double> mpc_x_vals(res.begin(), res.begin()+N);
          std::vector<double> mpc_y_vals(res.begin()+N, res.begin()+2*N);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          std::vector<double> next_x_vals(60);
          std::vector<double> next_y_vals(60);
          std::iota(next_x_vals.begin(), next_x_vals.end(), 0);
          std::transform(next_x_vals.begin(), next_x_vals.end(), next_y_vals.begin(), [&coeffs](double x){ return polyeval(coeffs, x); });

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


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
          std::this_thread::sleep_for(std::chrono::milliseconds(latency));
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
