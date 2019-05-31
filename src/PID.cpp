#include "PID.h"
#include <iostream>
#include <vector>
#include "math.h"


//Initialization for twiddle
std::vector<double> p;
std::vector<double> best_p;
double best_error = 100000; //best error
double tol = 0.001; //tolerance
int c = 0; //counter for span of time to check my controller
int c_tot = 500; //time span amount
int n=0; //parameter tuned
double dp[3]={0.01,0.0001,0.1}; //incrementer for PID constants -- ~1/10th of initial tuning parameters
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
  tot_error += pow(cte,2);	//increment error--always positive so doesn't cancel out
  
  //After c_tot # of samples, change PID
  if (c > c_tot) {
    std::cout << "Total error: " << tot_error << std::endl;
    //first sample set, set best error and best values
  	if (first) {
        best_error = tot_error;
      	best_p[n] = p[n];
        p[n]+=dp[n]; //increment
      	first = false;
  	} else { //second sample set, change dp based on if error improved or not
      	if (tot_error<=best_error && second){
        	best_error = tot_error;
        	best_p[n]=p[n];
        	dp[n]*=1.1;	//if increasing helped, keep increasing
        	p[n]+=dp[n];
      	}
      	else { //if increasing did not improve error
        	if (second) {
        		p[n]-=2*dp[n]; //if it did not help, go the opposite direction
        		second = false;
        	} else { //third try, see if continued improvement or not
      			if (tot_error<best_error){
        			best_error = tot_error;
        			best_p[n]=p[n];
        			dp[n]*=1.1;
        			p[n]+=dp[n];
      			} else {
        			p[n] += dp[n];
        			dp[n]*=0.8; //decrease step size
        			p[n]+=dp[n]; //re-increment
      			}
                n+=1; //after 3 times through, go to the next parameter to optimize
      			if (n>2){n=0;} //if went through all 3 constants, set back to the first
      			first = true; //reset flags
      			second = true;
            } //end of !second
        } //end of second
    } //end of !first
    
    c=0; //recount
    tot_error = 0; //reset
    
    //check if twiddle has sufficiently run its course
    double sum_dp = fabs(dp[0])+fabs(dp[1])+fabs(dp[2]);
    if (sum_dp<tol){
    //if (dp[n]<(tol/3)){
      std::cout << "DP: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
      twiddle = false; //turn off twiddle
      std::cout << "Twiddle Complete." << std::endl;
      std::cout << "Best PID values: " << best_p[0]<< ", " << best_p[1] << ", " << best_p[2] << std::endl;
    } 
    std::cout << "Current PID values: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "Best error= " << best_error << std::endl;
    std::cout << "Best PID values: " << best_p[0]<< ", " << best_p[1] << ", " << best_p[2] << std::endl;
  }//end cycle
} //end of Twiddle