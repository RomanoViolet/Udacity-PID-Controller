#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
  this->old_cte = 0.0;
}

PID::~PID()
{

}

void PID::Init(double Kp, double Ki, double Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->currentFrame = 0;

  std::cout << "PID Values: " << Kp << ", " << Ki << ", " << Kd << std::endl;


}

void PID::UpdateError(double cte) {
  this->p_error = cte;
  this->i_error = this->i_error + cte;
  this->d_error = cte - this->old_cte;
  this->old_cte = cte;
}

double PID::CalculateSteering() {
  double steeringValue;

  // update the total error
  this->UpdatetotalError();

  steeringValue = - (this->Kp * this->p_error) - (this->Kd * this->d_error) - (this->Ki * this->i_error);

  // update the number of frames seen so far
  this->currentFrame = this->currentFrame + 1;

  return(steeringValue);

}// double PID::CalculateSteering()


void PID::UpdatetotalError() {

  if (this->currentFrame > this->frameTheshold)
  {
    totalError += std::pow(this->p_error, 2.0);
  }

}//void PID::UpdatetotalError()


double PID::GetTotalError()
{
    return(this->totalError / this->currentFrame);
}// double PID::GetTotalError()


double PID::GetCurrentFrame()
{
    return(this->currentFrame);
  //return(currentFrame);

}// double PID::GetCurrentFrame()


