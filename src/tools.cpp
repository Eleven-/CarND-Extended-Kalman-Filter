#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	if((px+py)==0){
	    cout << "Error";
	}
	else{
	    float pxpy = px*px+py*py;
	    Hj(0,0) = px/sqrt(pxpy);
	    Hj(1,0) = -py/pxpy;
	    Hj(0,1) = py/sqrt(pxpy);
	    Hj(1,1) = px/pxpy;
	    Hj(2,0) = py*(vx*py-vy*px)/sqrt(pxpy*pxpy*pxpy);
	    Hj(2,1) = px*(vx*py-vy*px)/sqrt(pxpy*pxpy*pxpy);
	    Hj(2,2) = px/sqrt(pxpy);
	    Hj(2,3) = py/sqrt(pxpy);
	}
	//compute the Jacobian matrix

	return Hj;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
