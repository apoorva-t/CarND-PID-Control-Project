#ifndef PID_H
#define PID_H

 enum eval {first, second, third};

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dp[3];
  eval e;

  int paramIndex;

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
  double TotalError();

  void twiddleUpdate(double curr_cte);
};

#endif /* PID_H */
