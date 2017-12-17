#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double old_cte;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * number of frames after which the error will be compputed
   */
   const unsigned frameTheshold = 0;

   /*
    * frame number
    */
   unsigned currentFrame = 0;

   /*
    * total error
    */
   double totalError = 0.0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  void UpdatetotalError();

  /*
   * return the total error over a specified number of frames
   */
  double GetTotalError();

  /*
   * Calculate the new values for the steering wheel
   */
  double CalculateSteering();

  /*
   * Get the current frame
   */
  double GetCurrentFrame();

};


#endif /* PID_H */
