#include "Robotics.h"

MatrixXf getBlist()
{
    double h1 = 0.0515;
    double h2 = 0.19707;
    double h3 = 0.02999;
    double h4 = 0.0155;
    double w1 = 0.02993;
    double w2 = 0.0915;
    double w3 = 0.0515;
    double w4 = 0.0845;

    VectorXf B1(6);
    VectorXf B2(6);
    VectorXf B3(6);
    VectorXf B4(6);
    VectorXf B5(6);
    VectorXf B6(6);
    B1 << 0, 1, 0, (w1+w2+w3+w4), 0, 0;
    B2 << -1, 0, 0, 0, (w1+w2+w3+w4), -(h2+h3+h4);
    B3 << -1, 0, 0, 0, (w1+w2+w3+w4), -(h4+h4);
    B4 << 0, 0, 1, -h4, 0, 0;
    B5 << -1, 0, 0, 0, w4, -h4;
    B6 << 0, 0, 1, 0, 0, 0;

    MatrixXf Blist(6,6);
    Blist << B1, B2, B3, B4, B5, B6;
    return Blist;
}

MatrixXf getM()
{
    double h1 = 0.0515;
    double h2 = 0.19707;
    double h3 = 0.02999;
    double h4 = 0.0155;
    double w1 = 0.02993;
    double w2 = 0.0915;
    double w3 = 0.0515;
    double w4 = 0.0845;

    MatrixXf M(4,4);
    M <<    0, 0, 1, w1+w2+w3+w4,
            1, 0, 0, 0,
            0, 1, 0, h1+h2+h3+h4,
            0, 0, 0, 1;
    return M;
}

bool Nearzero(float value){
    if(value<0.000001)
        return true;
    else
        return false;
}

bool iszero(MatrixXf T) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (abs(T(i, j)) > 0.000001) {
                return false;
            }
        }
    }
    return true;
}


MatrixXf MatrixExp3(MatrixXf so3mat){
    MatrixXf R(3, 3);
    R << 1, 0, 0, 
        0, 1, 0, 
        0, 0, 1;
    VectorXf omgtheta = so3ToVec(so3mat);
    if(!Nearzero(omgtheta.norm())){
        float theta = omgtheta.norm();
        MatrixXf omgmat(3, 3);
        omgmat = so3mat/theta;
        R = R + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    }
    return R;
}

MatrixXf MatrixExp6(MatrixXf se3mat) {
    MatrixXf T(4, 4);
    T << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    MatrixXf EYE(3, 3);
    EYE << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    VectorXf omgtheta = so3ToVec(se3mat);
    if (Nearzero(omgtheta.norm())) {
        T.block<3, 1>(0, 3) = se3mat(seq(0, 2), 3);
    }
    else {
        float   theta = omgtheta.norm();
        MatrixXf so3mat = VecToso3(omgtheta);
        MatrixXf omgmat  = se3mat(seq(0, 2), seq(0, 2))/theta;
        
        T.block<3, 3>(0, 0) = MatrixExp3(se3mat(seq(0, 2), seq(0, 2)));
        T.block<3, 1>(0, 3) = (EYE * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) * se3mat(seq(0, 2), 3) / theta;
    }
    return T;
}

MatrixXf MatrixLog3(MatrixXf R){
    float acosinput = (R.trace() - 1) / 2;
    MatrixXf so3mat(3, 3);
    if (acosinput >=1){
        so3mat(0,0) = 1;
        so3mat(1,1) = 1;
        so3mat(2,2) = 1;
    }
    else if (acosinput <= -1){
        VectorXf omg(3);
        if(!Nearzero(1 + R(2,2))){
            omg(0) = (0 + R(0, 2)) / sqrt(2 * (1 + R(2, 2))) * M_PI;
            omg(1) = (0 + R(1, 2)) / sqrt(2 * (1 + R(2, 2))) * M_PI;
            omg(2) = (1 + R(2, 2)) / sqrt(2 * (1 + R(2, 2))) * M_PI;
        }
        else if (!Nearzero(1 + R(1,1))) {

            omg(0) = (0 + R(0, 1)) / sqrt(2 * (1 + R(1, 1))) * M_PI;
            omg(1) = (1 + R(1, 1)) / sqrt(2 * (1 + R(1, 1))) * M_PI;
            omg(2) = (0 + R(2, 1)) / sqrt(2 * (1 + R(1, 1))) * M_PI;
        }
        else{

            omg(0) = (1 + R(0, 0)) / sqrt(2 * (1 + R(0, 0))) * M_PI;
            omg(1) = (0 + R(1, 0)) / sqrt(2 * (1 + R(0, 0))) * M_PI;
            omg(2) = (0 + R(2, 0)) / sqrt(2 * (1 + R(0, 0))) * M_PI;
        }
        so3mat = VecToso3(omg);

    }
    else{
        float theta = acos(acosinput);
        so3mat = theta * (R - R.transpose()) / (2 * sin(theta));
    }
    return so3mat;
}

MatrixXf MatrixLog6(MatrixXf T) {
    MatrixXf R(3, 3);
    MatrixXf EYE(3, 3);
    MatrixXf ZERO(3, 3);
    EYE << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixXf expmat(4,4);
    R = T.block<3, 3>(0, 0);
    MatrixXf omgmat = MatrixLog3(R);
    if (iszero(omgmat)) {
        expmat <<
            0, 0, 0, T(0, 3),
            0, 0, 0, T(1, 3),
            0, 0, 0, T(2, 3),
            0, 0, 0, 0;
    }
    else {
        float theta = acos((R.trace() - 1) / 2);
        expmat.block<3, 3>(0, 0) = omgmat;
        expmat.block<3, 1>(0, 3) = (EYE - omgmat / 2 + (1 / theta - 1/tan(theta / 2) / 2) * omgmat * omgmat / theta) * T.block<3, 1>(0, 3);
        expmat.block<1, 4>(3, 0) << 0, 0, 0, 0;
    }

    return expmat;


}



MatrixXf Adjoint(MatrixXf T){
    MatrixXf R(3, 3);
    R = T.block<3, 3>(0, 0);
    MatrixXf AdT(6, 6);
    VectorXf p(3);
    p = T(seq(0, 2), 3);
    MatrixXf zeros(3, 3);
    zeros << 0, 0, 0,
        0, 0, 0,
        0, 0, 0;

    MatrixXf K = VecToso3(p) * R;
    AdT.block<3, 3>(0, 0) = R;
    AdT.block<3, 3>(0, 3) = zeros;
    AdT.block<3, 3>(3, 0) = K;
    AdT.block<3, 3>(3, 3) = R;
    return AdT;
}

VectorXf so3ToVec(MatrixXf so3mat) {
    VectorXf omega(3);
    omega(0) = so3mat(2,1);
    omega(1) = so3mat(0,2);
    omega(2) = so3mat(1,0);
    return omega;
}

VectorXf se3ToVec(MatrixXf se3mat) {
    VectorXf V(6);
    V(0) = se3mat(2, 1);
    V(1) = se3mat(0, 2);
    V(2) = se3mat(1, 0);
    V(3) = se3mat(0, 3);
    V(4) = se3mat(1, 3);
    V(5) = se3mat(2, 3);
    return V;
}

MatrixXf VecTose3(VectorXf V){
    MatrixXf se3mat(4, 4);
    VectorXf zeros(4);
    zeros << 0, 0, 0, 0;
    se3mat.block<3, 3>(0, 0) = VecToso3(V(seq(0, 2)));
    se3mat.block<3, 1>(0, 3) = V(seq(3, 5));
    se3mat.block<1, 4>(3, 0) = zeros;
    return se3mat;
}

MatrixXf VecToso3(VectorXf omg){
    MatrixXf so3mat(3, 3);
    so3mat << 0, -omg[2], omg[1],
        omg[2], 0, -omg[0],
        -omg[1], omg[0], 0;
    return so3mat;
}

MatrixXf JacobianBody(MatrixXf Blist, VectorXf thetalist) {
    //I'll presume that size(thetalist) is 6
    MatrixXf Jb = Blist;
    MatrixXf T(4, 4);
    T << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    VectorXf alist(6);
    for (int i = thetalist.rows()-2; i >= 0; i--) {//i indicates the index of the arm
        alist = (-1) * Blist(seq(0, 5), i + 1) * thetalist(i + 1);
        T = T * MatrixExp6(VecTose3(alist));
        Jb.block<6,1>(0, i) = Adjoint(T) * Blist(seq(0, 5), i);
    }
    return Jb;
}

MatrixXf FKinBody(MatrixXf M, MatrixXf Blist, VectorXf thetalist) {
    //I'll presume that size(thetalist) is 6
    
    MatrixXf T = M;
    VectorXf alist(6);
    for (int i = 0; i < thetalist.rows(); i++) {
        alist = Blist(seq(0, 5), i) * thetalist(i);
        T = T * MatrixExp6(VecTose3(alist));
    }
    return T;
}