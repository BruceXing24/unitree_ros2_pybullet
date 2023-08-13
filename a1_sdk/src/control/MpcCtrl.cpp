/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/MpcCtrl.h"
#include "common/mathTools.h"


MpcCtrl::MpcCtrl(QuadrupedRobot *robModel){
    Vec6 s;
    Vec12 w, u;

    _mass = robModel->getRobMass();
    _pcb = robModel->getPcb();         // desired position
    _Ib = robModel->getRobInertial();
    _g << 0, 0, -9.81;

    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    _alpha = 0.001;
    _fricRatio = 0.4;
    s << 20, 20, 50, 450, 450, 450;


    _S = s.asDiagonal();
    _W = w.asDiagonal();
    _U = u.asDiagonal();
    
    _Fprev.setZero();   //previous force
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
    _a.setZero();
    _b.setZero();
    _horizon = 20;




}


// get paper-16 then 
void MpcCtrl::calMatrix_a(){
    for(int i(0); i < 4; ++i){
        _a.block(3, 3*i, 3, 3) = _B2G_RotMat;
        _a(3, 9) = 1 ;
        _a(4, 10) = 1;
        _a(5, 11) = 1;
        _a(11,12) = 1;
             }

}

// get paper-17 then 
void MpcCtrl::calMatrix_b(){
    const int num_legs=4;
    for (int i = 0; i < num_legs; ++i) {
        _b.block<3, 3>(6, i * 3) = _Ig.inverse() * skew(_feet_position.row(i));
        _b(9, i * 3) = 1/_mass;
        _b(10, i * 3 + 1) = 1/_mass;
        _b(11, i * 3 + 2) = 1/_mass;
    }    
}

void MpcCtrl::getTrajall(Vec12 trajInial,std::vector<float>  vx_vy_yaw){

 for(int i = 0; i < _horizon; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInial[j];
        for(int j = 0; j < 12; j++){
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + _mpc_dt * vx_vy_yaw[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + _mpc_dt * vx_vy_yaw[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + _mpc_dt * vx_vy_yaw[2];
        }
}
}

// get paper-15
void MpcCtrl::getIg(Mat3 rotM){
   _B2G_RotMat = rotM;
   _Ig = _B2G_RotMat*_Ib*_B2G_RotMat.transpose();
}

void MpcCtrl::getState(Vec3 rpy ,Vec3 Pxyz, Vec3 Wrpy, Vec3 Vxyz){
    _rpy  = rpy;
    _Pxyz = Pxyz;
    _Wrpy = Wrpy;
    _Vxyz = Vxyz;
}

void MpcCtrl::getFeetPosition(Vec34 feet_position){
    _feet_position = feet_position;
    
    
}

void MpcCtrl::run(){



    //1.set weights ,  get foot vector , get
    
    //2. QP setting

    //3. QP solver 


}



void MpcCtrl::qp_sovler(){



    //  1.get state  _ac _bc _ad _bd from continous to discrete , paper -16,17

    // 2. get _ad _bd then get A Bqp

    // 3. update trajectory

    // 4. add friction constrain to matrix

    // 5. get H,g in 31, 32

    // define a qp problem choose a sovler 

    Eigen::Matrix<double,13,1> x_0;       //初始状态

    // std::cout<<"pred_horizon=="<<_horizon<<std::endl;

    //  1.get state
    x_0 <<_rpy[0],_rpy[1],_rpy[2],
          _Pxyz[0],_Pxyz[1],_Pxyz[2],
          _Wrpy[0],_Wrpy[1],_Wrpy[2], 
          _Vxyz[0],_Vxyz[1],_Vxyz[2],
          -9.8;
    calMatrix_a();
    calMatrix_b();
    // 2. get _ad _bd then get A Bqp

    c2qp(_a,_b,_mpc_dt,_horizon);


    Eigen::Matrix<double,13,1> full_weight;
    for(int16_t i = 0; i < 12; i++)
        full_weight(i) = _weights[i];
    full_weight(12) = 0.f;
    _S = full_weight.asDiagonal();

    // update trajectory in robot_mpc.cpp
    // 3. update trajectory

  for(int16_t i = 0; i < _horizon; i++)
  {
    for(int16_t j = 0; j < 12; j++)
      _X_d(13*i+j,0) = trajAll[12*i+j];
  }

    // 3. add friction constrain to matrix



}


void MpcCtrl::c2qp(Mat13 Ac, Eigen::Matrix<double,13,12> Bc,double dt, int horizon)
{
  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;
  ABc.block(0,13,13,12) = Bc;
  ABc = dt*ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,12);

  Eigen::Matrix<double,13,13> powerMats[20];
  powerMats[0].setIdentity();
  for(int i = 1; i < horizon+1; i++) {
    powerMats[i] = Adt * powerMats[i-1];
  }

  for(int r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    for(int c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        int a_num = r-c;
        B_qp.block(13*r,12*c,13,12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }
  }










// void MpcCtrl::calConstraints(VecInt4 contact){

// }

// void MpcCtrl::solveQP(){

// }