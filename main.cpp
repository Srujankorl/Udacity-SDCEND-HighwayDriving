#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "spline.h"
#include "fsm.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // Code from the Lesson in Unit 6.Projet Q&A was used to implement the trajectory planning with the spline.h 
  // starting with the car in the lane 1
  
  int lane = 1; // 0 = left, 1 = middle, 2 = right  
  // defining a reference velocity variable to be able to later implement the calcultion of the points for the trajectory spline 
  double ref_vel = 0.0; //miles per hour

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            int prev_size = previous_path_x.size();

            if (prev_size > 0) {
                car_s = end_path_s;
            }
          
           //defining the boolean variables, to later determine from the sensorfusion data, if there is presence of other vehicles nearby
          bool car_right = false;
          bool car_left = false; 
          bool car_ahead = false;       
                    
           //Step 1. Prediction: Predicting the behavious of the vehicles around us
          
          //running a loop through the sensor fusion data where the 
          //"i" represents the ith vehicle and the 6th row has the lane information (d frenet coordinate) of the ith vehicle
          
          for (int i = 0; i < sensor_fusion.size(); i++) {
                float d_sensorfusion = sensor_fusion[i][6];
                //initializing the value of the other cars lane to an invalid value
                int car_lane_sensorfusion = -1;
            
                // estimating the lane of the other car from the sensorfusion d value
                if (d_sensorfusion > 0 && d_sensorfusion < 4) {
                  car_lane_sensorfusion = 0; //left lane
                } else if (d_sensorfusion > 4 && d_sensorfusion < 8) {
                  car_lane_sensorfusion = 1; //middle lane
                } else if (d_sensorfusion > 8 && d_sensorfusion < 12) {
                  car_lane_sensorfusion = 2;//right lane
                }
                //to ensure that we do not calculate with an invalid lane value
                if (car_lane_sensorfusion < 0) {
                  continue;
                }
                // calculating the speed of the other car using the sensorfusion values for its x and y directions
                double vx_sensorfusion = sensor_fusion[i][3]; //vx value from sensorfusion data
                double vy_sensorfusion = sensor_fusion[i][4]; //vy value from sensorfusion data
                double check_speed_sensorfusion = sqrt(vx_sensorfusion*vx_sensorfusion + vy_sensorfusion*vy_sensorfusion);
                double check_car_s_sensorfusion = sensor_fusion[i][5]; //s value from sensorfusion data
            
                // Predicting the value for s of the car after it has travelled with the speed calculated from the sensorfusion information
                check_car_s_sensorfusion += ((double)prev_size*0.02*check_speed_sensorfusion);
                
                //if sensorfusion information reveals that the car is in our lane
                if (car_lane_sensorfusion == lane) { //implies that the car is in our lane                 
                  
                  //the car is ahead of us and in the "danger zone", if the difference in s values 
                  //is lesser than 30 m and greater than 0 m                  
                  //setting the car_ahead boolean to true if those conditions are satisfied
                  if(check_car_s_sensorfusion > car_s && check_car_s_sensorfusion - car_s < 30){
                  car_ahead = true;
                  }
                } else if (car_lane_sensorfusion - lane == -1) { //implies that the car is in the lane left to us                  
                  //for this car to be in our danger zone, it should be 30m before or after us 
                  //setting the car_left boolean to true if those conditions are satisfied
                  if(car_s - 30 < check_car_s_sensorfusion && car_s + 30 > check_car_s_sensorfusion){
                  car_left = true;
                  }
                } else if ( car_lane_sensorfusion - lane == 1 ) { //implies that the car is in the lane right to us
                  //for this car to be in our danger zone, it should be 30m before or after us 
                  //setting the car_right boolean to true if those conditions are satisfied
                  if(car_s - 30 < check_car_s_sensorfusion && car_s + 30 > check_car_s_sensorfusion){
                  car_right = true ;
                  }
                }
            }
          
            // Step 2. Behavior: telling the car what to do in the above cases of encounter with other vehicles
            
            double speed_difference = 0;
            const double max_velocity = 49.5;
            const double max_acceleration = .224;
            
            if (car_ahead) { // the bbol Car ahead is true
              if (lane > 0 && !car_left) {
                // and if there is a left lane (lane>0) and no car is in the danger zone in the left lane.
                lane--; // Change the lane left
              } else if (lane != 2 && !car_right){
                // and if there is a right lane (lane!=2) and no car is in the danger zone in the right lane.
                lane++; // Change the lane right
              } else {
                speed_difference -= max_acceleration;//Reduce the difference in speed (decelerate)
              }
            } else {
              if (lane != 1) { // In this case, our car is not in the middle lane
                if ((lane == 0 && !car_right) || (lane == 2 && !car_left)) { 
                  //there are no cars in the danger zone in the center lane
                  lane = 1; // Change lane back to the center.
                }
              }
              if (ref_vel < max_velocity) {//if our velocity is less than the max velocity, increase the speed difference (accelerate)
                speed_difference += max_acceleration;
              }
            }
          
          // Step 3. Trajectory Planning: with the help of the spline library and the explanation from the lesson 6.Project Q&A
           /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
            // Creating a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // later interpolate waypoints with spline and fill in more waypoints          

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            //deciding which point to be used as a reference
            // If previous path size is almost empty, use the car as starting reference
            if (prev_size < 2) {
                
                //using the points that make a tangent to the car x and y coordinates
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            // else we will be using the last 2 points of the previous path as a reference
            else {
                // now overwriting the reference state as the end points of the previous path
                
                // The last point on the previous path
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];
                // The second-last point on the previous path
                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                //adding these points as "anchor points" the the ptsx and ptsy arrays
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            //now on frenet adding 30m evenly spaced points ahead of the previously created anchor points
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            
            //changinf the frame of reference to the car frame of ref
            for (int i=0; i<ptsx.size(); i++) {
                
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;

                ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            //creating a spline
            tk::spline s;

            // Setting points on to the spline, with their separation distance calculated by using the reference velocity
            s.set_points(ptsx, ptsy);

            // Defining the actual (x,y) points which will be used by the path planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Using the previous path points from the previous iteration, so that we dont have to create all the points new
            for (int i=0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculating the spacing between these points to obtain the reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            // Filling up the remaining points (after using the previous path points) in the path planner 
            
            for (int i=0; i <= 50-previous_path_x.size(); i++) {
              
              //varying the reference velocity depending on the values for speed difference determined in
              //the behaviour step
              ref_vel += speed_difference;
              if ( ref_vel > max_velocity ) {
                ref_vel = max_velocity;
              } else if ( ref_vel < max_acceleration ) {
                ref_vel = max_acceleration;
              }

                double N = (target_dist/(0.02*ref_vel/2.24));
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point;
                double x_ref = x_point;
                double y_ref = y_point;

                //Rotating the reference coordinates back to world coordinates
                x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
                x_point += ref_x;
                y_point += ref_y;
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }
            json msgJson;
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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