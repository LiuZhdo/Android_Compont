//
// Created by admin on 2025/10/13.
//

#ifndef IMP_ABRQ_CONTROL_CURVE_POINT_H
#define IMP_ABRQ_CONTROL_CURVE_POINT_H

double Cal_Curve_Point(
        double vehicle_x, double vehicle_y,
        // 第一段曲线参数
        double AAn1, double BBn1, double CCn1, double DDn1,
        double AAe1, double BBe1, double CCe1, double DDe1, double skk1, double skk2,
        // 第二段曲线参数
        double AAn2, double BBn2, double CCn2, double DDn2,
        double AAe2, double BBe2, double CCe2, double DDe2, double skk3,
        // 输出参数
        double *ref_n, double *ref_e,
        double *heading, double *curvature, double *distance);

#endif //IMP_ABRQ_CONTROL_CURVE_POINT_H
