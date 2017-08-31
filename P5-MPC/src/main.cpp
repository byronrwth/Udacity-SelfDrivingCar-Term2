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

  std::cout << "main: polyeval: result: " << result << std::endl;


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
  std::cout << "main: polyfit: result: " << result << std::endl;


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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];

          std::cout << "main: parsing waypoints: ptsx: " << std::endl;
          for (auto i = ptsx.begin(); i != ptsx.end(); ++i)
              std::cout << *i << ' ';
          std::cout << " \n " << std::endl;

          vector<double> ptsy = j[1]["ptsy"];

          std::cout << "main: parsing waypoints: ptsy: " << std::endl;
          for (auto i = ptsy.begin(); i != ptsy.end(); ++i)
              std::cout << *i << ' ';
          std::cout << " \n " << std::endl;

          double px = j[1]["x"];
          double py = j[1]["y"];
          std::cout << "main: parsing car at: px= " << px << " py= " << py << "\n" << std::endl;


          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          std::cout << "main: parsing car with psi and velocity: psi= " << psi << " v= " << v << "\n" << std::endl;

          /* ============  QA ================*/
          
          for ( int i = 0; i < ptsx.size(); i++) 
          {
              //shift car reference angle to 90 degrees
              double shift_x = ptsx[i] - px ;
              double shift_y = ptsy[i] - py ;

              ptsx[i] = ( shift_x * cos( 0 - psi) - shift_y * sin( 0 - psi)) ;
              ptsy[i] = ( shift_x * sin( 0 - psi) + shift_y * cos( 0 - psi)) ;
          } 
          std::cout << "main: ======= re-coordinate ======================= \n " << std::endl;

          std::cout << "main: car-coordinated waypoints: ptsx: " << std::endl;
          for (auto i = ptsx.begin(); i != ptsx.end(); ++i)
              std::cout << *i << ' ';
          std::cout << " \n " << std::endl;

          std::cout << "main: coordinated waypoints: ptsy: " << std::endl;
          for (auto i = ptsy.begin(); i != ptsy.end(); ++i)
              std::cout << *i << ' ';
          std::cout << " \n " << std::endl;


          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
          std::cout << "main: ptsx_transform: " << ptsx_transform << std::endl;

          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
          std::cout << "main: ptsy_transform: " << ptsy_transform << std::endl;


          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
          std::cout << "main: coeffs: " << coeffs << std::endl;

          //calculate cte and epsi
          double cte = polyeval(coeffs, 0); // in car's coordiante, x = y= 0
          std::cout << "main: cte: " << cte << std::endl;
          //double cte = polyeval(coeffs, x) - y;
          //std::cout << "main: cte: " << cte << " , x: " << x << " ,coeffs: " << coeffs << " ,y: " << y << std::endl;

          // init at psi == 0, px, py==0 ?
          //double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px, 2))
          double epsi = -atan(coeffs[1]);
          std::cout << "main: epsi = -atan(" << coeffs[1]<< "): " << epsi << std::endl;



          double steer_value = j[1]["steering_angle"]; 
          std::cout << "main: steer_value: " << steer_value << std::endl;

          double throttle_value = j[1]["throttle"];
          std::cout << "main: throttle_value: " << throttle_value << std::endl;

          Eigen::VectorXd state(6);

          // Recall the equations for the model:
          // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
          // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
          // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
          // v_[t] = v[t-1] + a[t-1] * dt
          // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
          // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt


          // no latency, in current car's coordinate, x=y=psi=0
          //state << 0, 0, 0, v, cte, epsi;

          // need to use lantency in case speed = 100
          double t_latency = 0.1 ;
          double delta = steer_value ;
          double a = throttle_value;
          double Lf = 2.67 ;


          double latency_x = v * t_latency ;
          double latency_y = 0 ;
          double latency_psi = v /Lf * delta * t_latency ;
          double latency_v = v + a * t_latency ;
          double latency_cte = cte + v * sin(epsi) * t_latency ;
          double latency_epsi = epsi + v * delta * t_latency / Lf ;

          state << latency_x, latency_y, latency_psi, latency_v, latency_cte, latency_epsi;

          /* ============ end  QA ================*/

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
#if 0
          double steer_value;
          double throttle_value;
          
          auto coeffs = polyfit(ptsx, ptsy, 1);
          double cte = polyeval(coeffs, px) - py;
          double epsi = psi - atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << x, y, psi, v, cte, epsi;

          steer_value = ;
          
          throttle_value =  ;
#endif

          auto vars = mpc.Solve(state, coeffs);  

          
          //Display the waypoints/reference line
          vector<double> next_x_vals;  // ? std::vector<double> v;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 25;

          for ( int i =1; i < num_points; i++)
          {
              next_x_vals.push_back(poly_inc * i) ;
              next_y_vals.push_back(polyeval(coeffs, poly_inc * i)) ;
          }
          
          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for ( int i =2; i < vars.size(); i++ )
          {
              if ( i %2 == 0)
              {
                  mpc_x_vals.push_back(vars[i]);
              }
              else
              {
                  mpc_y_vals.push_back(vars[i]);
              }
          }


          

          double solved_steer = -vars[0]/(deg2rad(25) * Lf) ;
          double solved_throttle = vars[1] ;
          std::cout << "main: Solve: solved_steer = " << solved_steer << " ,solved_throttle = " << throttle_value << std::endl;



          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = solved_steer ; //steer_value;
          msgJson["throttle"] =  solved_throttle ;  //throttle_value;



          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;



          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

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
