#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"
#include <fstream>      // std::ifstream, std::ofstream

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
  ofstream outfile;
  outfile.open ("out.csv");
  outfile << "x_est" << "\t";
  outfile << "y_est" << "\t";
  outfile << "v_est" << "\t";
  outfile << "yaw_est" << "\t";
  outfile << "yawrate_est" << "\t";
  outfile << "vx_est" << "\t";
  outfile << "vy_est" << "\t";
  outfile << "x_meas" << "\t";
  outfile << "y_meas" << "\t";
  outfile << "x_gt" << "\t";
  outfile << "y_gt" << "\t";
  outfile << "v_gt" << "\t";
  outfile << "yaw_gt" << "\t";
  outfile << "yawrate_gt" << "\t";
  outfile << "vx_gt" << "\t";
  outfile << "vy_gt" << "\t";
  outfile << "NIS_lidar" << "\t";
  outfile << "NIS_radar" << "\t";
  outfile << "rmse_x" << "\t";
  outfile << "rmse_y" << "\t";
  outfile << "rmse_vx" << "\t";
  outfile << "rmse_vy" << "\n";

  h.onMessage([&ukf,&tools,&estimations,&ground_truth,&outfile](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
        float yaw_gt;     // added
        float yawrate_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
        iss >> vy_gt;
        iss >> yaw_gt;
        iss >> yawrate_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);
        float v_gt;
        v_gt = sqrt(vx_gt*vx_gt + vy_gt*vy_gt);
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
    	  ukf.ProcessMeasurement(meas_package);    	  

    	  //Push the current estimated x,y positon from the Kalman filter's state vector

    	  VectorXd estimate(4);

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

    	  // --- output for visualization ---
    	  //output the estimation
    	  outfile << ukf.x_(0) << "\t"; //pos1 - est
    	  outfile << ukf.x_(1) << "\t"; //pos2 - est
    	  outfile << ukf.x_(2) << "\t"; //vel_abs -est
    	  outfile << ukf.x_(3) << "\t"; //yaw_angle -est
    	  outfile << ukf.x_(4) << "\t"; //yaw_rate -est
        outfile << v1 << "\t"; //v1 -est
        outfile << v2 << "\t"; //v1 -est
        //output the measurements
        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            //output the estimation
            outfile << meas_package.raw_measurements_(0) << "\t";    //p1 - meas
            outfile << meas_package.raw_measurements_(1) << "\t"; //p2 - meas
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            //output the estimation in the cartesian coordinates
            float ro = meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);
            outfile << ro * cos(phi) << "\t";    //p1_meas
            outfile << ro * sin(phi) << "\t";    //p2_meas
        }
        //output the ground truth packages
        outfile << x_gt << "\t"; //p1 - GT
        outfile << y_gt << "\t";    //p2 - GT
        outfile << v_gt << "\t";    //v_abs - GT
        outfile << yaw_gt << "\t";    //yaw - GT
        outfile << yawrate_gt << "\t";    //yaw_dot - GT
        outfile << vx_gt << "\t";    //v1 - GT
        outfile << vy_gt << "\t";    //v2 - GT

        //output Normalized Innovation Squared (NIS)
        outfile << ukf.nis_lidar_ << "\t"; // NIS lidar
        outfile << ukf.nis_radar_ << "\t"; // NIS radar

        //output Root Mean Squared Error
        outfile << RMSE(0) << "\t"; // rmse_x
        outfile << RMSE(1) << "\t"; // rmse_y
        outfile << RMSE(2) << "\t"; // rmse_vx
        outfile << RMSE(3) << "\n"; // rmse_vy

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

  outfile.close();
}






















































































