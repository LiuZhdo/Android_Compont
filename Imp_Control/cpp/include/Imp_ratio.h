
// Created by admin on 2025/3/3.

#ifndef IMP_RATIO_H
#define IMP_RATIO_H

#include <iostream>
#include <string>
#include <cmath>
#include "Eigen/Dense"
#include <vector>
#include "android_log.h"

using Matrix = Eigen::MatrixXd;

typedef struct Imp_State{
    double x;
    double y;
    double psi;
    double v;
    double imp_rol;
    double dimpsi;
    double errImp;
}imp_State;

typedef struct ago_data{
    double err;
    double errImp;
    double dpsi;
    double dimpsi;
    double rol;
    double improl;
    double delta;
    double Ptime;
    double PtimeOff;
}data_ago;

typedef struct imp_data{
    double imp_K0;
    double imp_K1;
    double imp_K2;
    double imp_U0;
    double q00;
    double q22;
    double r00;
    double imp_max;
}imp_data;

class LQR_imp_psi {
public:
    imp_data impData{};
    LQR_imp_psi(uint a1, double a2, double a3);
    // void save_ago_state(double err,double errImp,double dpsi,double dimpsi,double rol,double improl,double delta,double Ptime,double PtimeOff);
    void update_imp_state(double x, double y, double psi, double v);
    void update_car_state(double x,double y,double psi,double v);
    void Update_A_B_matrix(double L11, double L22, double L33,Eigen::Vector3d *diff_state);
    void Update_Q_R_matrix(double q00,double q11,double q22,double r00,double r11,int go_heading);
    double CALC(Eigen::Vector3d *dif_state, Eigen::MatrixXd *ptr_K, double errImp_max);

private:
    double tolerance = 1e-6;
    double dt = 0.1;
    // double gam = 0;
    uint max_num_iteration = 700;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd M;
    Eigen::MatrixXd P;

    Imp_State cur_state{};
    Imp_State imp_state{};
    ago_data ago{};
    void Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
               const Matrix &M,Matrix *ptr_K);
    void Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,Matrix *ptr_K);
    void Solve(Matrix *ptr_K);

};

#endif // IMP_RATIO_H
