#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      
      auto s = hasData(std::string(data));
      if (s != "") {

        /*
main(): s=["telemetry",{"lidar_measurement":"L\t0.2611\t-0.9724\t1477010443000000\t0.1800\t-0.8300\t5.0000\t0.0000","radar_measurement":"R\t0.1401\t-1.3702\t6.0960\t1477010443000000\t0.1800\t-0.8300\t5.0000\t0.0000","hunter_x":"-10.0000","hunter_y":"0.0000","hunter_heading":"0.0000"}]
        */
      	std::cout << "main(): s=" << s << std::endl;

        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::cout << " main(): data 42 start, event=telemetry : " << std::endl;

          string sensor_measurment = j[1]["sensor_measurement"];
          //string lidar_measurment = j[1]["lidar_measurement"];
          //string radar_measurment = j[1]["radar_measurement"];

          //std::string sensor_measurment = j[1].get<std::string>();
          //std::cout << "main(): lidar_measurment" << lidar_measurment << std::endl;
          //std::cout << "main(): radar_measurment" << radar_measurment << std::endl;

          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);  //  ??
          //istringstream iss_lidar(lidar_measurment);  //  ??
    	    long long timestamp;
  
    	    // reads first element from the current line
    	    string sensor_type;
          iss >> sensor_type;
    	    //iss_lidar >> sensor_type;

    	    if (sensor_type.compare("L") == 0) {
                std::cout << " got Lidar measurement ! " << std::endl;
      	    		meas_package.sensor_type_ = MeasurementPackage::LASER;
            		meas_package.raw_measurements_ = VectorXd(2);
            		float px; //0.261
      	    		float py; //-0.9724 
                iss >> px;
                iss >> py;
            		//iss_lidar >> px;
            		//iss_lidar >> py;
            		meas_package.raw_measurements_ << px, py;
            		iss >> timestamp; //1477010443000000
                //iss_lidar >> timestamp; //1477010443000000
            		meas_package.timestamp_ = timestamp;

                // gt = t0.1800\t-0.8300\t5.0000\t0.0000
            } else if (sensor_type.compare("R") == 0) {
                std::cout << " got Radar measurement ! " << std::endl;
      	    		meas_package.sensor_type_ = MeasurementPackage::RADAR;
            		meas_package.raw_measurements_ = VectorXd(3);
            		float ro;
      	    		float theta;
      	    		float ro_dot;
                iss >> ro; // 0.1401
                iss >> theta; // -1.3702
                iss >> ro_dot; //6.0960
            		//iss_radar >> ro; // 0.1401
            		//iss_radar >> theta; // -1.3702
            		//iss_radar >> ro_dot; //6.0960
            		meas_package.raw_measurements_ << ro, theta, ro_dot;
            		iss >> timestamp; //1477010443000000
                //iss_radar >> timestamp; //1477010443000000
            		meas_package.timestamp_ = timestamp;
                // gt = \t0.1800\t-0.8300\t5.0000\t0.0000

                // since the last 4 elements of ground truth are identical from lidar and radar, so save only once
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt; 
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);
             
             //Call ProcessMeasurment(meas_package) for Kalman filter
    	    ukf.ProcessMeasurement(meas_package);    	  
   
    	    //Push the current estimated x,y positon from the Kalman filter's state vector
   
    	    VectorXd estimate(4);

        /*
        x_ <<
            px_p;
            py_p;
            v_p;
            yaw_p;
            yawd_p;
        */
        double p_x = ukf.x_(0);
    	  double p_y = ukf.x_(1);
    	  double v  = ukf.x_(2);
    	  double yaw = ukf.x_(3);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
    	  estimations.push_back(estimate);

    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        
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























































































