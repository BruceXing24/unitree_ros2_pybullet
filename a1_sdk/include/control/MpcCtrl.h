/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef MPCCTRL_H
#define MPCCTRL_H

#include "common/mathTypes.h"
#include "common/unitreeRobot.h"

#define BIG_NUMBER 5e10

class MpcCtrl{
public:
    MpcCtrl(QuadrupedRobot *robModel);
    void getTrajall(Vec12 trajInial,std::vector<float> vx_vy_yaw);
    void getIg(Mat3 rotM);
    void getState(Vec3 rpy ,Vec3 Pxyz, Vec3 Wrpy, Vec3 Vxyz);
    void getFeetPosition(Vec34 feet_position);

private:

void calMatrix_a();
void calMatrix_b();
void run();
void qp_sovler();
void c2qp(Mat13 Ac, Eigen::Matrix<double,13,12> Bc,double dt, int horizon);


Eigen::VectorXd trajAll;


            // discrete
Mat13 _a;
Eigen::Matrix<double,13,12> _b;

// Vec13 _x,_x_dot;
int _horizon;
double _mpc_dt;
Vec3 _rpy , _Pxyz, _Wrpy, _Vxyz;
Vec34 _feet_position;
Eigen::Matrix<double,-1,13> A_qp;
Eigen::Matrix<double,-1,-1> B_qp;
Eigen::Matrix<double,13,12> Bdt;
Eigen::Matrix<double,13,13> Adt;
Eigen::Matrix<double,25,25> ABc,expmm;
Mat3 _Ib,_Ig, _B2G_RotMat;
float _weights[12];
double _mass, _alpha, _fricRatio;
Mat13 _S;
Eigen::VectorXf _X_d, _U_b;

//Matrix for qp
Mat12 _G, _W, _U;
// Mat6 _S;
Vec6 _bd;
Vec3 _g;
Vec3 _pcb;
Vec12 _F, _Fprev, _g0T;
Eigen::MatrixXd _CE, _CI;
Eigen::VectorXd _ce0, _ci0;
Eigen::Matrix<double, 6 , 12> _A;
Eigen::Matrix<double, 5 , 3 > _fricMat;

};

#endif  // MPCCTRL_H