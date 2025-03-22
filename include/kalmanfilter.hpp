#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;


class  KalmanFilter
{

public: 

  // Data output, n-length
  MatrixXd meanVector; // "x"	nx1 vector

  // Sensors measurements, m-length
  MatrixXd measurementVector; // "z" mx1 vector

  // Prediction
  MatrixXd updateMatrix; // "F" nxn matrix

  // What would we expect +the matrix to be if we
  //  got PERFECT data
  MatrixXd extractionMatrix; // "H" mxn matrix

  // Covariance of our data
  MatrixXd covarianceMatrixX; // "P" nxn matrix

  // Measurement Covariance Matrix
  MatrixXd covarianceMatrixZ; // "R" mxm matrix

  MatrixXd moveVector;

  KalmanFilter(int variables, int measurements,
    MatrixXd updateMatrix,
    MatrixXd extractionMatrix,
    MatrixXd covarianceMatrixX,
    MatrixXd covarianceMatrixZ,
    MatrixXd moveVector);

    void KalmanUpdate(MatrixXd meas);
    void KalmanPredict();
  
};
