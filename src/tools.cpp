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
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if(estimations.size()==0){
	    cout << "Error estimation vector size = 0";
	    return rmse;
	}
	if(estimations.size()!=ground_truth.size()){
	    cout << "Error estimation != ground truth size";
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
	}

	//calculate the mean
	// ... your code here
	rmse = rmse/estimations.size();

	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();
	//return the result
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	
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
