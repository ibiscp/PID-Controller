#ifndef PID_H
#define PID_H
#include <vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  std::vector<double> K;

  /*
  * Defined
  */
  int iteration;
  double last_error;
  std::vector<long long int> timestamp;
  double reference;
  std::vector<double> delta_K;
  std::vector<double> error_squared;
  int twiddle_parameter;
  int twiddle_tries;

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
  void Init();//double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double GetControl();

  /*
  * Calculate the next Twiddle parameters
  */
  void UpdateTwiddle();
};

#endif /* PID_H */
