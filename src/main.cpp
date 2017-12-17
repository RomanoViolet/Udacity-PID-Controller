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

void resetSimulator(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


int main(int argc, char* argv[])
{
  uWS::Hub h;

  PID pid;

  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;
  unsigned numberOfFramestoMonitor = 0;


  if(argc==1)
  {
    //default parameters
	Kp = 0.3;
    	Ki = 0.002;
    	Kd = 1.1;
      numberOfFramestoMonitor = 5000;
  }
  else
  {
    // read in the value of Kp from a tuner wrapper
      std::string KpString = argv[1];
      Kp =  atof(KpString.c_str());

      // read in the value of Ki from the tuner wrapper
      std::string KiString = argv[2];
      Ki =  atof(KiString.c_str());

      // read in the value of Kd from the tuner wrapper
      std::string KdString = argv[3];
      Kd =  atof(KdString.c_str());

      /* read in the number of frames after which the simulation is considered 'complete'
       * and it is therefore reset to the beginning
      */
      std::string nFramestoMonitorString = argv[4];
      numberOfFramestoMonitor =  static_cast<unsigned>(atoi(nFramestoMonitorString.c_str()));
  }






  // initialize PID controller values
  pid.Init(Kp, Ki, Kd);
  // TODO: Initialize the pid variable.

  h.onMessage([&pid, Kp, Ki, Kd, numberOfFramestoMonitor](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    double setThrottle;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          // reset the simulator, if this is a fresh start
          if(pid.GetCurrentFrame() == 0)
          {
            resetSimulator(ws);
          }


          std::cout << "Actual Steering Value at Car: " << angle << "\t";

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          //pid.Init()
          // update error terms to be used for proportional, integral, and differential controllers
          pid.UpdateError(cte);

          // steering needs to be supplied in radians
          steer_value = (pid.CalculateSteering() - deg2rad(angle));

          // modulate speed depending upon the steering angle
          setThrottle = (std::exp(-std::fabs(1.5*cte))-0.1) * 0.4;

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value Requested: " << steer_value << std::endl;

          if( abs(cte) > 2.0 )
          {
            // reset the simulator
            resetSimulator(ws);
            std::cout << "Simulator aborted after "<< pid.GetCurrentFrame() << " frames due to large Cross Track Error: " << pid.GetTotalError() << " and PID values: " << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << std::endl;
            return (-1);
          }

          if ( pid.GetCurrentFrame() > numberOfFramestoMonitor )
          //if ( pid.GetCurrentFrame() > 500 )
          {
            // reset the simulator
            resetSimulator(ws);
            std::cout << "Simulator Reset due to reaching the set number of frames with total error: " << pid.GetTotalError() << " and PID values: " << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << std::endl;
            return (0);
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = setThrottle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
