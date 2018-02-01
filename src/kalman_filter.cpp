#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;  //2x4 x 4x1 = 2x1
  VectorXd y = z - z_pred;    //2x1
  
  MatrixXd Ht = H_.transpose(); //4x2
  MatrixXd S = H_ * P_ * Ht + R_; //2x4 x 4x4 x 4x2 = 2x2
  MatrixXd Si = S.inverse();    // 2x2
  MatrixXd PHt = P_ * Ht;       // 4x4 x 4x2 = 4x2
  MatrixXd K = PHt * Si;      // 4x2 x 2x2 = 4x2
  
  //new estimate  
  x_ = x_ + (K * y);    // 4x2 x 2x1 = 4x1
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;   // 4x4 x 4x4 = 4x4
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho, psi, rho_dot;
  
  rho = sqrt(px*px+py*py);
  
  // To avoid divide by zero when calculating rho_dot
  if(rho < 0.0001)
    rho = 0.0001;
  
  psi = atan2(py,px);
  
  // To normalize psi within -pi to +pi
  if(psi > M_PI)
    psi = psi - 2*M_PI;
  else if(psi < -M_PI)
    psi = psi + 2*M_PI;
    
  rho_dot = ((px*vx) + (py*vy))/rho;
       
  VectorXd z_pred(3);
  z_pred << rho, psi , rho_dot;  
  VectorXd y = z - z_pred;    // 3x1
  
  // To normalize psi within -pi to +pi
  if(y[1] > M_PI)
    y[1] = y[1] - 2*M_PI;
  else if(y[1] < -M_PI)
    y[1] = y[1] + 2*M_PI;

  MatrixXd Ht = H_.transpose();   //T(3x4) = 4x3
  MatrixXd S = H_ * P_ * Ht + R_; //3x4 x 4x4 x 4x3 = 3x3
  MatrixXd Si = S.inverse();    // 3x3
  MatrixXd PHt = P_ * Ht;     // 4x4 x 4x3 = 4x3
  MatrixXd K = PHt * Si;      // 4x3 x 3x3 = 4x3
  
  //new estimate  
  x_ = x_ + (K * y);    //4x3 x 3x1 = 4x1
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}


