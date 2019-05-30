#include "PID.h"
#include <iostream>
#include <vector>


//Initialization for twiddle
  std::vector<double> p;
  std::vector<double> best_p;
	double best_error = 100000; //best error
	const double tol = 0.05; //tolerance
	int c = 0; //counter for span of time to check my controller
	int c_tot = 50; //time span amount
	int n=0; //parameter tuned
	double dp[3]={0.3,0.001,0.1}; //incrementer for PID constants
	bool first = true; //first run through twiddle
	bool second = true; //second run through twiddle 
	double tot_error = 0; //total error 
	bool twiddle = true; //continuing to twiddle 


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  
  p = {Kp_, Ki_, Kd_}; //p vector used for twiddle
  best_p = p; //parameter vector to keep track of the best PID constants in twiddle
  
  //start with zero error
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
//new p,i,d error values based on cte
  d_error = cte - p_error; //constant equal to the change in error- before updating p_error, it is the last error

  p_error = cte; //proportional to error
  
  i_error += cte; //integral error keeps track of total error
  
  if (twiddle){
  	Twiddle(cte); //call twiddle
  }
}

double PID::TotalError() {
  double total_error = -(p[0]*p_error) - (p[1]*i_error) - (p[2]*d_error);
  return total_error;
}

void PID::Twiddle(double cte){
  
  //every time twiddle is called, increment the counter and total error
  c+=1; //increment counter
  tot_error += abs(cte);	//increment error--always positive so doesn't cancel out
  
  //After c_tot # of samples, change PID
  if (c > c_tot){
    //first sample set, set best error and best values
  	if (first){
    	best_error = tot_error;
      	best_p[n] = p[n];
        p[n]+=dp[n]; //increment
      	tot_error = 0;
      	first = false;
  	}
    //second sample set, change dp based on if error improved or not
  	else if (!first && second) {
      if (tot_error<best_error){
        best_error = tot_error;
        dp[n]*=1.1;	//if increasing helped, keep increasing
        p[n]+=dp[n];
      }
      else {
        dp[n]*=-2*dp[n]; //if it did not help, set opposite
        p[n]+=dp[n];
      }
        second=false; 
    }
    //third sample set, see if continued improvement or not
    else {
      if (tot_error<best_error){
        best_error = tot_error;
        best_p[n]=p[n];
        dp[n]*=1.1;
        p[n]+=dp[n];
      }
      else{
        p[n]-=dp[n]; //go back to previous
        dp[n]*=0.8; //decrease step size
        p[n]+=dp[n]; //re-increment
      }
      n+=1; //after 3 times through, go to the next parameter to optimize
      if (n>2){n=0;} //if went through all 3 constants, set back to the first
      first = true;
      second = true;
      best_error = 100000;
    }

    c=0; //recount
    tot_error = 0; //reset
    
    //check if twiddle has sufficiently run its course
    double sum_dp = dp[0]+dp[1]+dp[2];
    if (sum_dp<tol){
      twiddle = false; //turn off twiddle
      std::cout << "Best PID values: " << best_p[0]<< ", " << best_p[1] << ", " << best_p[2] << std::endl;
    } 
  }
	std::cout << "Interrum PID values: " << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << std::endl;
} //end of Twiddle