
// Created by LZD on 2025/04/12.

#include "Imp_ratio.h"
#include "AgguideIm.h"

void LQR_imp_psi::Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                        const Matrix &M, Matrix *ptr_K) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
        Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
        M.rows() != Q.rows() || M.cols() != R.cols()) {
        return;
    }
    Matrix AT = A.transpose();
    Matrix BT = B.transpose();
    Matrix MT = M.transpose();
    P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance) {
        Matrix P_next =
                AT * P * A -
                (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
    }
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

void LQR_imp_psi::Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                        Matrix *ptr_K) {
    Matrix M = Matrix::Zero(Q.rows(), R.cols());
    this->Solve(A, B, Q, R, M, ptr_K);
}

void LQR_imp_psi::Update_Q_R_matrix(double q00, double q11, double q22, double r00, double r11,int go_heading) {
    double q2 = 100,r0 = 80;
    if(go_heading == 0){
        // 参数自适应车速 车速过快调节力度需要小
        if(abs(cur_state.v)<0.5 && abs(cur_state.v)>0.01){  // 0.036~2.5km/h
            q2 = 130;
            r0 = 80;
            LOGI("curv < 0.5,curv > 0.01");
        }
        if(abs(cur_state.v)<1.2 && abs(cur_state.v)>0.5){  // 2.5~4.3km/h
            q2 = 120;
            r0 = 90;
            LOGI("curv < 1.25,curv > 0.5");
        }
        if(abs(cur_state.v)<1.75 && abs(cur_state.v)>1.2){  // 4.3~6.3km/h
            q2 = 110;
            r0 = 95;
            LOGI("curv < 1.75,curv > 1.25");
        }
        if(abs(cur_state.v)<2.5 && abs(cur_state.v)>1.75){  // 7.2~9km/h
            q2 = 95;
            r0 = 135;
            LOGI("curv < 2.5,curv > 1.75");
        }
        if(abs(cur_state.v)<3 && abs(cur_state.v)>2.5){  // 9~10.8km/h
            q2 = 85;
            r0 = 150;
            LOGI("curv < 3,curv > 2.5");
        }
        if(abs(cur_state.v)>3){                                     // >9.9km/h
            q2 = 70;
            r0 = 165;
            LOGI("curv > 3");
        }
        q22 = q2 + 100 - q22;    // q2 默认值100
        r00 = r0 + 100 - r0;
        if(cur_state.v < 0){
            if(cur_state.v < -1){
                q22 = 120;   r00 = 90;
            }
            q11 = q11 + 5;    q22 = q22 - 10;    r00 = r00 + 10;
        }
    }
    LOGI("imp_ratio:q0:%f,q2:%f,r0:%f", q00, q22, r00);
    Q(0,0)=q00;
    Q(1,1)=q11;
    Q(2,2)=q22;
    R(0,0)=r00;
    R(1,1)=r11;
    impData.q00 = q00;
    impData.q22 = q22;
    impData.r00 = r00;
}

void LQR_imp_psi::Solve(Matrix *ptr_K) {
    this->Solve(A, B, Q, R, ptr_K);
}

void LQR_imp_psi::Update_A_B_matrix(double L11, double L22, double L33, Eigen::Vector3d *diff_state)  {
//void LQR::Update_A_B_matrix(double L11, double L22, double L33)  {
    // 倒车时车速符号取反了。(倒车时车速为负值)
    // gam = (*diff_state)(0) - (*diff_state)(1);
    A(0,0)=1.0;
    // A(1,0)=cur_state.v* cos(gam)*dt/L33;
    // A(1,1)=1.0-cur_state.v* cos(gam)*dt/L33;
    A(1,0)=cur_state.v*dt/L33;
    A(1,1)=1.0-cur_state.v*dt/L33;
    A(2,1)=imp_state.v*dt;
    A(2,2)=1.0;

    B(0,0)=cur_state.v*dt/L11;
    // B(1,0)=-cur_state.v*cos(gam)*dt*L22/(L11*L33);
    B(1,0)=-cur_state.v*dt*L22/(L11*L33);
}

double LQR_imp_psi::CALC(Eigen::Vector3d *dif_state, Eigen::MatrixXd *ptr_K, double errImp_max) {

    // 尝试 five small tricks: (重点前三个)
    //      1.引入历史农具横偏信息，判断农具是否已经入线， 入线与在线参数不同；
    //      2.组合坡利用农机横滚角变化作为预测信息调节相关参数；
    //      3.引入农机实时横滚角信息 朝上坡打转向控制力度更大 朝下坡打转向控制力度更小，如何更好地刹住农机惯性；
    //      4.引入农机航向角变化率信息；
    //      5.引入农机的实际转角与期望转角的差值、期望角度变化量，如何更好地刹住电机惯性。
    this->Solve(A, B, Q, R, ptr_K);
    Matrix KK = *ptr_K;
    Matrix UU = -(*ptr_K) * (*dif_state);

    int one_flag = 0;
    double errImp_upper = KK(0,1) * (*dif_state)(1) + KK(0,2) * (*dif_state)(2);
    if(ImplementType == 1){ errImp_upper =(*dif_state)(2); one_flag = 1;}
    if(abs(errImp_upper) <= abs((*dif_state)(2))) {errImp_upper = (*dif_state)(2); one_flag = 1;}

    impData.imp_max = 0;
    if(abs(errImp_upper) >= abs(errImp_max)){
        errImp_upper = errImp_max * errImp_upper/abs(errImp_upper);
        if(one_flag == 0){
            UU(0, 0) = -(KK(0,0) * (*dif_state)(0) + errImp_upper);
        }
        else{
            UU(0, 0) = -(KK(0,0)*(*dif_state)(0) + KK(0,1)*(*dif_state)(1) + KK(0,2)*errImp_upper);
        }
        impData.imp_max = errImp_upper;
    }
    LOGI("20250930:base:KK1=%lf,KK2=%lf,KK3=%lf,UU1=%lf,UU2=%lf,errImp_upper:%f",
         KK(0, 0), KK(0, 1), KK(0, 2),UU(0, 0),UU(1, 0),errImp_upper);
    impData.imp_K0 = KK(0, 0);
    impData.imp_K1 = KK(0, 1);
    impData.imp_K2 = KK(0, 2);
    impData.imp_U0 = UU(0, 0);
    return UU(0, 0);
}

LQR_imp_psi::LQR_imp_psi(uint a1, double a2, double a3){
    tolerance=a2;
    max_num_iteration=a1;
    dt=a3;

    A = Matrix::Zero(3, 3);
    B = Matrix::Zero(3, 2);
    Q = Matrix::Zero(3, 3);
    R = Matrix::Zero(2, 2);
    P = Matrix::Zero(3, 3);
}

void LQR_imp_psi::update_car_state(double x, double y, double psi, double v) {
    cur_state={x,y,psi,v};
    LOGI("cur_state={%lf,%lf,%lf,%lf}",cur_state.x,cur_state.y,cur_state.psi,cur_state.v);
}

void LQR_imp_psi::update_imp_state(double x, double y, double psi, double v) {
    imp_state={x,y,psi,v};
    LOGI("imp_state={%lf,%lf,%lf,%lf}",imp_state.x,imp_state.y,imp_state.psi,imp_state.v);
}

/*void LQR_imp_psi::save_ago_state(double err,double errImp,double dpsi,double dimpsi,double rol,double improl,double delta,double Ptime,double PtimeOff){
    ago={err,errImp,dpsi,dimpsi,rol,improl,delta,Ptime,PtimeOff};
}*/

