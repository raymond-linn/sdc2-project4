#include <iostream>
#include <vector>
#include <numeric>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

static vector<double> previous_cte;

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void PID::UpdateError(double cte) {

  double tau_p = Kp;
  double tau_d = Kd;
  double tau_i = Ki;
  double diff_cte = 0;
  double int_cte = 0;

  if(previous_cte.size()!= 0){
    diff_cte = cte - previous_cte.back();
  }
  else{
    diff_cte = cte ;
  }
  previous_cte.push_back(cte);

  for (auto& n : previous_cte)
    int_cte += n;

  p_error = tau_p * cte ;
  d_error = tau_d * diff_cte;
  i_error = tau_i * int_cte;

}

double PID::TotalError() {
  double steer = -p_error - d_error - i_error;
  return steer;
}

void PID::OptimizeParameters(std::vector<double> p,
                             std::vector<double> dp,
                             double best_error,
                             int control_state,
                             int idx){

/*
  std::cout << "PID Error: " << TotalError() << "\t" << "Best Error: " << best_error << std::endl;
  if (control_state == 0) {
    // initialize the p and best error
    p[idx] += dp[idx];
    best_error = TotalError();
    control_state = 1;
  }
  else if (control_state == 1){
    if(TotalError() < best_error){
      best_error = TotalError();
      p[idx] *= 1.1;
      // update to rotate three controls
      idx = (idx+1) % 3;
      p[idx] += dp[idx];
      control_state = 1;
    }
    else {
      p[idx] -= 2 * dp[idx];
      if (p[idx] < 0) {
        p[idx] = 0;
        idx = (idx +1) % 3;
      }
      control_state = 2;
    }
  }
  else { // for last
    if (TotalError() < best_error) {
      best_error = TotalError();
      dp[idx] *= 1.1;
      idx = (idx + 1) % 3;
      p[idx] += dp[idx];
      control_state = 1;
    }
    else {
      p[idx] += dp[idx];
      dp[idx] *= 0.9;
      idx = (idx + 1) % 3;
      p[idx] += dp[idx];
      control_state = 1;
    }
  }
  // intialize the pid with new parameter
  Init(p[0], p[1], p[2]);
  */

}
