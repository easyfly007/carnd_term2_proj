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

#include "util.h"

// for convenience
using json = nlohmann::json;
using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

extern size_t N;
extern double dt;
extern bool verbose;
extern int order;
extern double Lf;

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
    cout << endl << endl;
    cout << "onMessage: " << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> mapcoord_ptsx = j[1]["ptsx"];
          vector<double> mapcoord_ptsy = j[1]["ptsy"];
          assert(mapcoord_ptsx.size() == mapcoord_ptsy.size());

          // the initial value, e.g., the current state of the car
          double mapcoord_px0    = j[1]["x"];
          double mapcoord_py0    = j[1]["y"];
          double mapcoord_psi0   = j[1]["psi"];
          double mapcoord_v0     = j[1]["speed"];
          double mapcoord_delta0 = j[1]["steering_angle"];
          double mapcoord_a0     = j[1]["throttle"];

          // as the speed received from the simulator, the unit is 

          // note that ptsx and ptsy are from map coordinate, 
          // we need to transfer it into car coordinate for future MPC implementation
          Eigen::VectorXd carcoord_ptsx(mapcoord_ptsx.size());
          Eigen::VectorXd carcoord_ptsy(mapcoord_ptsy.size());
          for (size_t i = 0; i < mapcoord_ptsx.size(); i ++)
          {
            double mapcoord_ptx = mapcoord_ptsx[i];
            double mapcoord_pty = mapcoord_ptsy[i];

            double dx = mapcoord_ptx - mapcoord_px0;
            double dy = mapcoord_pty - mapcoord_py0;
            double carcoord_ptx = dx * cos(mapcoord_psi0) + dy * sin(mapcoord_psi0);
            double carcoord_pty = - dx * sin(mapcoord_psi0) + dy * cos(mapcoord_psi0);

            // double carcoord_ptx = cos(mapcoord_psi0) * mapcoord_ptx + sin(mapcoord_psi0) * mapcoord_pty 
            //   - cos(mapcoord_psi0) * mapcoord_px0 - sin(mapcoord_psi0) * mapcoord_py0;
            // double carcoord_pty = -sin(mapcoord_psi0) * mapcoord_ptx + cos(mapcoord_psi0) * mapcoord_pty 
            //   + sin(mapcoord_psi0) * mapcoord_px0 - cos(mapcoord_psi0) * mapcoord_py0;
            carcoord_ptsx(i) = carcoord_ptx;
            carcoord_ptsy(i) = carcoord_pty;
          }
          Eigen::VectorXd coeffs = polyfit(carcoord_ptsx, carcoord_ptsy, order);
          double carcoord_px0 = 0.0;
          double carcoord_py0 = 0.0; 
          double carcoord_psi0 = 0.0;
          double carcoord_v0 = mapcoord_v0;
          // double carcoord_cte0  = carcoord_py0 - polyeval(coeffs, carcoord_px0);
          double carcoord_cte0  = polyeval(coeffs, carcoord_px0) - carcoord_py0;
          // question: why it's y_ref - y_state? I think it should be y_state - y_ref for cte

          // coeffs[1] is the derive of the reference path at position x = 0
          double carcoord_epsi0 = carcoord_psi0 - atan(coeffs[1]) ;
          double carcoord_delta0 = mapcoord_delta0;
          double carcoord_a0 = mapcoord_a0;
          // now px0, py0, psi0, v0, cte0, epsi0 are the initial state and at car coordinate 
          if (verbose)
          {
            cout << "the initial states at map coordinate are: " << endl;
            cout << "    px0=" << mapcoord_px0 << ", py0=" << mapcoord_py0 << ", psi0=" << mapcoord_psi0
              << ", v0 = " << mapcoord_v0 << ", psi0=" << mapcoord_psi0 << endl;

            cout << "the initial states at car coordinate are: " << endl;
            cout << "    px0 =" << carcoord_px0 << ", py0=" << carcoord_py0 << ", psi0=" << carcoord_psi0 
            << ", v0=" << carcoord_v0 << ", cte0=" << carcoord_cte0 << ", epsi0=" << carcoord_epsi0 << endl;

            cout << "the reference ptsx and ptsy in map coordinate:" << endl;
            for (size_t i = 0; i < mapcoord_ptsx.size(); i ++)
              cout << "    (" << mapcoord_ptsx[i] << ", " << mapcoord_ptsy[i] << ")";
            cout << endl;
            cout << "the reference ptsx and ptsy in car coordinate:" << endl;
            for (int i = 0; i < carcoord_ptsx.size(); i ++)
              cout << "    (" << carcoord_ptsx(i) << ", " << carcoord_ptsy(i) << ")";
            cout << endl;
          }

          // now consider latency, when we provide control for the simulator,
          // there's always 0.1s delay 
          // so we will will try to fit the time after the latency time
          double latency_time = 0.1;
          double latency_px0 = carcoord_px0 + carcoord_v0 * cos(carcoord_psi0) * latency_time;
          double latency_py0 = carcoord_py0 + carcoord_v0 * sin(carcoord_psi0) * latency_time;
          double latency_psi0 = carcoord_psi0 - carcoord_delta0 * carcoord_v0 * latency_time / Lf;
          double latency_v0   = carcoord_v0 + carcoord_a0 * latency_time;
          double latency_cte0 = carcoord_cte0 + carcoord_v0 * sin(carcoord_epsi0) * latency_time;
          // double latency_epsio= carcoord_epsi0 + carcoord_v0 * carcoord_delta0*  latency_time / Lf; 
          double latency_epsio= carcoord_epsi0 - carcoord_v0 * carcoord_delta0*  latency_time / Lf; 
          // replace delta with - delta in the equation
          
          Eigen::VectorXd state0(6);
          state0 << latency_px0, latency_py0, latency_psi0, latency_v0, latency_cte0, latency_epsio;
          auto result = mpc.Solve(state0, coeffs);
		      if (verbose)
            cout << "Solve result size = " << result.size() << endl;
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // x, y, psi, v, cte, epsi, delta, a
          // 0  1  2    3  4    5     6      7
          double steer_value = result[0] / deg2rad(25);
          // as in the simulator, turn left means negative steering value, turn right means positive steering value
          // in the car coordinate, turn left means positive value, turn right means negative value,
          // 
          double throttle_value = result[1];
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          if (verbose)
          {
            cout << "  **  steering angle = " << steer_value << endl;
            cout << "  **  throttle value = " << throttle_value << endl;
          }
          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (size_t i = 0; i < 10; i ++)
          {
            double carcoord_ptx1 = result[ 2 + i * 2];
            double carcoord_pty1 = result[ 2 + i * 2 + 1];
            // note that ptx1 and pty1 are in car coordinate
            // switch back to map coordinate
            // double mapcoord_ptx2 = mapcoord_px0 + carcoord_ptx1 * cos(mapcoord_psi0) - carcoord_pty1 * sin(mapcoord_psi0);
            // double mapcoord_pty2 = mapcoord_py0 + carcoord_ptx1 * sin(mapcoord_psi0) + carcoord_pty1 * cos(mapcoord_psi0);
          	// mpc_x_vals.push_back(mapcoord_ptx2);
          	// mpc_y_vals.push_back(mapcoord_pty2);

            // as we need to put the car coordinate mpc points to the simulator
            mpc_x_vals.push_back(carcoord_ptx1);
            mpc_y_vals.push_back(carcoord_pty1);
          }
          if (verbose)
          {
          	cout << "the mpc x val at mapcoord are: "  << endl;
          	for (size_t i = 0; i < mpc_x_vals.size() && i < 10; i ++)
          	    cout << "  " << mpc_x_vals[i];
          	cout << endl;
          	cout << "the mpc y val at mapcoord are: "<< endl;
          	for (size_t i = 0; i < mpc_y_vals.size() && i < 10; i ++ )
          		cout << "  " << mpc_y_vals[i];
          	cout << endl;
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i = 0; i < 20; i ++)
          {
            double ptx1 = 2.0 * i;
            double pty1 = polyeval(coeffs, ptx1);
            next_x_vals.push_back(ptx1);
            next_y_vals.push_back(pty1);
          }
          if (verbose)
          {
          	cout << "the size for next_x_vals = " << next_x_vals.size() << endl;
          	for (size_t i = 0; i < next_x_vals.size() && i < 10; i ++ )
          	    cout << " " << next_x_vals[i];
          	cout << endl;
          	cout << "the size for next_y_vals = " << next_y_vals.size() << endl;
          	for (size_t i =0; i < next_y_vals.size() && i < 10; i ++)
	          	cout << " " << next_y_vals[i];
	        cout << endl;
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          
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
