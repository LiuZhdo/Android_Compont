//
// Created by admin on 2025/05/21.
//

#ifndef IMP_ALINETRACK_IMP_DELTA_H
#define IMP_ALINETRACK_IMP_DELTA_H


#include "VehicleNavControl.h"

/*curx cury:农机位置  单位 m
 *impx impy:农具位置  单位 m
 * cur_psi imp_psi 农机 农具航向 rad
 * curv impv 农机 农具速度 m/s
 * rol imp_rol 农机 农具 横滚角 rad
 * pitch imp_pitch  农机 农具俯仰角 rad
 * double Omiga  车身转速,顺时针为正 rad/s
 * double WAS   当前真实前轮转角值 rad
 * imp_mode 农具导航模式
 * double *lzd_delta  模式22求解的目标期望转角
 * double RQ_ago 上一时刻计算的目标前轮转角 rad*/
double Imp_ESO(double curx, double cury, double impx, double impy, double cur_psi, double imp_psi, double curv, double impv, double rol,
               double imp_rol, double pitch, double imp_pitch, double Omiga, double WAS, int imp_mode, double *delta, double *RQ_ago);

void LADRCcontrolForImplement(const VehicleNavParasIn_T &data1, const VehicleNavDatasIn_T &data2, VehicleNavOut_T* data3);

#endif //IMP_ALINETRACK_IMP_DELTA_H
