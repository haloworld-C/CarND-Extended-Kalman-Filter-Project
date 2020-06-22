#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    * TODO: Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    // check the validity of the following inputs:
    // * the estimation vector size should not be zero
    // * the estimation vector size should equal ground truth size
    if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    for(int i = 0; i < int(estimations.size()); i++){
	    VectorXd residual = estimations[i] - ground_truth[i];

	    // cofficient - wise multiplication
	    residual = residual.array()*residual.array();
	    rmse += residual;
    }

    // calculate the meam
    rmse = rmse/estimations.size();

    // calculate sqrt of mean
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  // caculate taylor linear factor
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-caculate
	float c1 = px * px + py * py;
 	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	if (fabs(c1) < 0.0001){
		cout <<"CaculateJacobian() Error: Division by Zero!!"<< std::endl;
	}

	MatrixXd H_L(3,4);
	H_L << px/c2, py/c2, 0, 0,
		-py/c1, px/c1, 0, 0,
		py*(vx*py - vy*px)/c3, px*(vy*px - vx*py)/c3, px/c2, py/c2;

	return H_L;
}
