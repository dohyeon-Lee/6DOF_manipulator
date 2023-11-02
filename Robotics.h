#define _USE_MATH_DEFINES
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

bool Nearzero(float value);

MatrixXf MatrixExp3(MatrixXf so3mat);

MatrixXf MatrixExp6(MatrixXf se3mat);

MatrixXf MatrixLog3(MatrixXf R);

MatrixXf Adjoint(MatrixXf T);

VectorXf so3ToVec(MatrixXf so3mat);

VectorXf se3ToVec(MatrixXf se3mat);

MatrixXf VecTose3(VectorXf V);

MatrixXf VecToso3(VectorXf omg);

MatrixXf JacobianBody(MatrixXf Blist, VectorXf thetalist);

MatrixXf FKinBody(MatrixXf M, MatrixXf Blist, VectorXf thetalist);