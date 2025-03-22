#include "Eigen/Dense"
#include <iostream>
#include "kalmanfilter.hpp"

using Eigen::MatrixXd;

KalmanFilter::KalmanFilter(int variables, int measurements,
                           MatrixXd updateMatrix,
                           MatrixXd extractionMatrix,
                           MatrixXd covarianceMatrixX,
                           MatrixXd covarianceMatrixZ,
                           MatrixXd moveVector) : updateMatrix(updateMatrix), extractionMatrix(extractionMatrix), moveVector(moveVector), covarianceMatrixX(covarianceMatrixX), covarianceMatrixZ(covarianceMatrixZ)

{

    meanVector.Zero(variables, 1);
    measurementVector.Zero(variables, 1);
}

void KalmanFilter::KalmanPredict()
{
    MatrixXd x_next;
    MatrixXd P_next;
    MatrixXd F_trans;

    x_next = updateMatrix * meanVector + moveVector;
    P_next = updateMatrix * covarianceMatrixX;
    F_trans = updateMatrix.transpose();
    P_next = P_next * F_trans;

    meanVector = x_next;
    covarianceMatrixX = P_next;

    delete &x_next;
    delete &P_next;
    delete &F_trans;
}

void KalmanFilter::KalmanUpdate(MatrixXd meas)
{
    MatrixXd y;
    MatrixXd S;
    MatrixXd extTran;
    MatrixXd K;
    MatrixXd SinV;
    MatrixXd x_next;
    MatrixXd P_next;

    measurementVector = meas;

    y = extractionMatrix * meanVector;
    y = measurementVector - y;

    S = extractionMatrix * covarianceMatrixX;
    extTran = extractionMatrix.transpose();
    S = S * extTran;
    S = S + covarianceMatrixZ;

    SinV = S.inverse();
    K = covarianceMatrixX * extTran;
    K = K * SinV;

    x_next = K * y;
    x_next = meanVector + x_next;

    P_next = covarianceMatrixX + extTran;
    P_next = P_next + SinV;
    P_next = P_next + extractionMatrix;
    P_next = P_next + covarianceMatrixX;

    P_next = covarianceMatrixX - P_next;

    meanVector = x_next;
    covarianceMatrixX = P_next;

    delete &y;
    delete &S;
    delete &extTran;
    delete &K;
    delete &SinV;
    delete &x_next;
    delete &P_next;
}