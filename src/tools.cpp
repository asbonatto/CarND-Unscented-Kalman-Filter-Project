#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  VectorXd rmse(4);
  
  // Initialize with zeros, otherwise it can blow upper_bound
  rmse << 0,0,0,0;
  
  // Exception handling
  if (estimations.size() == 0){
      std::cout << "No measurements found" << std::endl;
      return rmse;

  }
  if (estimations.size() != ground_truth.size()){
      std::cout << "Dimension mismatch" << std::endl;
      return rmse;
  }
  
  
  
  VectorXd diff;
  for (int i = 0; i < estimations.size(); i++){
      
      diff = estimations[i] - ground_truth[i];
      diff = diff.array()*diff.array();
      rmse += diff;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

double Tools::normalize_angles(double angle){
  double angle_c = cos(angle);
  double angle_s = sin(angle);
  
  if (fabs(angle_c) < 1E-4) {
      if(angle_s > 0){
           angle = M_PI/2;
      }else{
          angle = -M_PI/2;
      }
  }else{
      angle = atan2(angle_s, angle_c);
  }
  return angle
}