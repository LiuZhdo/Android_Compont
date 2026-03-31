//
// Created by admin on 2025/10/13.
//

#include "include/Agguide.h"
#include "include/AgguideIm.h"
#include "include/VehicleNavControl.h"
#include "include/android_log.h"
#include "Eigen/Dense"
#include <unsupported/Eigen/Polynomials>
#include <cmath>

// 使用Eigen多项式求解器替代原来的特征值方法
int GetRootQr_polynomial(double dblCoef[], double mina, double maxb, double *root);

double getcurvdis(double minsk, double maxsk, double x, double y,
                  double *ns, double *es, double *ns1, double *es1,
                  double *ns2, double *es2, double *ns3, double *es3,
                  double &sk_curve,
                  double AAn1, double BBn1, double CCn1, double DDn1,
                  double AAe1, double BBe1, double CCe1, double DDe1)
{
    double sk = minsk;
    double skk1 = maxsk;

    // 曲线参数方程的展开式，保存其系数
    // n_s = A_n + B_n * (s - s_k) + C_n * (s - s_k)^2 + D_n * (s - s_k)^3;
    // e_s = A_e + B_e * (s - s_k) + C_e * (s - s_k)^2 + D_e * (s - s_k)^3;
    // f(s) = (e(s)-x)^2+ (n(s)-y)^2
    // df(s)/ds = 2*(e(s)-x)*(de(s)/ds)+2*(n(s)-y)*(dn(s)/ds) = 0
    // (e(s)-x)*(de(s)/ds)+(n(s)-y)*(dn(s)/ds) = 0

    // (n(s)-y)函数展开项系数
    double AAn = DDn1;                                           // 三次项系数
    double BBn = CCn1 - 3 * DDn1*sk;                              // 二次项系数
    double CCn = 3 * DDn1*sk*sk - 2 * CCn1*sk + BBn1;              // 一次项系数
    double DDn = AAn1 - BBn1*sk + CCn1*sk*sk - DDn1*sk*sk*sk - y;   // 常数项

    // dn(s)/ds函数展开项系数
    double EEn = 3 * DDn1;                                       // 导函数(最高此次项系数)二次项系数
    double FFn = 2 * CCn1 - 6 * DDn1*sk;                          // 导函数一次项
    double GGn = BBn1 - 2 * CCn1*sk + 3 * DDn1*sk*sk;              // 导函数常数项

    // (n(s)-y)*(dn(s)/ds)的乘积项系数
    double a1 = AAn*EEn;                        // 五次项系数
    double a2 = AAn*FFn + BBn*EEn;              // 四次项系数
    double a3 = AAn*GGn + BBn*FFn + CCn*EEn;    // 三次项系数
    double a4 = BBn*GGn + CCn*FFn + DDn*EEn;    // 二次项系数
    double a5 = CCn*GGn + DDn*FFn;              // 一次项系数
    double a6 = DDn*GGn;                        // 常数项

    // (e(s)-x)函数展开项系数
    double AAe = DDe1;
    double BBe = CCe1 - 3 * DDe1*sk;
    double CCe = 3 * DDe1*sk*sk - 2 * CCe1*sk + BBe1;
    double DDe = AAe1 - BBe1*sk + CCe1*sk*sk - DDe1*sk*sk*sk - x;

    // de(s)/ds函数展开项系数
    double EEe = 3 * DDe1;
    double FFe = 2 * CCe1 - 6 * DDe1*sk;
    double GGe = BBe1 - 2 * CCe1*sk + 3 * DDe1*sk*sk;

    // (e(s)-x)*(de(s)/ds)的乘积项系数
    double b1 = AAe*EEe;
    double b2 = AAe*FFe + BBe*EEe;
    double b3 = AAe*GGe + BBe*FFe + CCe*EEe;
    double b4 = BBe*GGe + CCe*FFe + DDe*EEe;
    double b5 = CCe*GGe + DDe*FFe;
    double b6 = DDe*GGe;

    // 存储数据数组
    double dblCoef[5] = { 0 };

    // 定义0
    double zero = 1e-16;
    double tmpab = a1 + b1;

    // 曲率过小接近直线时，设置tmpab = 1e-16
    if(tmpab <zero && tmpab >= 0)
    {
        tmpab = zero;
    }

    if(tmpab > -zero && tmpab <= 0)
    {
        tmpab = -zero;
    }

    // 多项式的系数数组   (e(s)-x)*(de(s)/ds)+(n(s)-y)*(dn(s)/ds) = 0
    dblCoef[0] = (a6 + b6) / (tmpab);    // 首一降序 常数项
    dblCoef[1] = (a5 + b5) / (tmpab);    // 首一降序 一次项
    dblCoef[2] = (a4 + b4) / (tmpab);    // 首一降序 二次项
    dblCoef[3] = (a3 + b3) / (tmpab);    // 首一降序 三次项
    dblCoef[4] = (a2 + b2) / (tmpab);    // 首一降序 四次项
    //五次项系数为1

    double S_curv = 0.0;                 // 曲线最近点

    // 使用Eigen多项式求解器替代原来的特征值方法
    int ret = GetRootQr_polynomial(dblCoef, sk, skk1, &S_curv);

    // 注释掉直线判断和处理部分
    /*
    if(fabs(CCn1) < 0.001 && fabs(DDn1) < 0.001 && fabs(CCe1) < 0.001 && fabs(DDe1) < 0.001)
    {
        // 直线处理代码已注释掉
    }
    */

    // 如果找到实数解 确定的垂足参数s_curv
    if (ret == 1)
    {
        // 确定S_curv对应的曲线位置点坐标
        *ns = AAn1 + BBn1*(S_curv - sk) + CCn1*(S_curv - sk)*(S_curv - sk) + DDn1*(S_curv - sk)*(S_curv - sk)*(S_curv - sk);
        *es = AAe1 + BBe1*(S_curv - sk) + CCe1*(S_curv - sk)*(S_curv - sk) + DDe1*(S_curv - sk)*(S_curv - sk)*(S_curv - sk);

        *ns1 = 3 * DDn1*S_curv*S_curv + (2 * CCn1 - 6 * DDn1*sk)*S_curv + (BBn1 + 3 * DDn1*sk*sk - 2 * CCn1*sk);
        *ns2 = 6 * DDn1*S_curv + 2 * CCn1 - 6 * DDn1*sk;
        *ns3 = 6 * DDn1;

        *es1 = 3 * DDe1*S_curv*S_curv + (2 * CCe1 - 6 * DDe1*sk)*S_curv + (BBe1 + 3 * DDe1*sk*sk - 2 * CCe1*sk);
        *es2 = 6 * DDe1*S_curv + 2 * CCe1 - 6 * DDe1*sk;
        *es3 = 6 * DDe1;
        sk_curve = S_curv;
        double dis = sqrt((*ns - y)*(*ns - y) + (*es - x)*(*es - x));
        return dis;
    }
    else
    {
        // 没有找到实数解，返回错误
        return -1;
    }
}

// 使用Eigen多项式求解器替代原来的特征值方法
int GetRootQr_polynomial(double dblCoef[], double mina, double maxb, double *root)
{
    int ret = 0;

    // 构造多项式系数（从高次到低次）
    // 原dblCoef数组是降序排列：[0]常数项, [1]一次项, [2]二次项, [3]三次项, [4]四次项
    // 我们需要构造：s^5 + dblCoef[4]*s^4 + dblCoef[3]*s^3 + dblCoef[2]*s^2 + dblCoef[1]*s + dblCoef[0]
    Eigen::VectorXd polyCoeffs(6);
    polyCoeffs << dblCoef[0],  // 常数项
            dblCoef[1],  // 一次项系数
            dblCoef[2],  // 二次项系数
            dblCoef[3],  // 三次项系数
            dblCoef[4],  // 四次项系数
            1.0;         // 五次项系数

    // 创建多项式求解器
    Eigen::PolynomialSolver<double, Eigen::Dynamic> polySolver;
    polySolver.compute(polyCoeffs);

    // 获取所有根
    const Eigen::Matrix<std::complex<double>, Eigen::Dynamic, 1>& roots = polySolver.roots();

    // 寻找在范围内的实根
    const double IMAGINARY_THRESHOLD = 1e-12;
    const double BOUNDARY_TOLERANCE = 1e-6;

    std::vector<double> valid_roots;

    for (auto root_complex : roots) {
        // 检查是否为实根
        if (std::abs(root_complex.imag()) < IMAGINARY_THRESHOLD) {
            double real_root = root_complex.real();

            // 检查是否在有效范围内
            if (real_root >= mina - BOUNDARY_TOLERANCE &&
                real_root <= maxb + BOUNDARY_TOLERANCE) {
                valid_roots.push_back(real_root);
            }
        }
    }

    if (valid_roots.empty()) {
        return 0;  // 没有找到有效根
    }

    // 如果有多个实根，选择最接近区间中点的那个
    double range_mid = (mina + maxb) * 0.5;
    double best_root = valid_roots[0];
    double min_distance = std::fabs(best_root - range_mid);

    for (size_t i = 1; i < valid_roots.size(); ++i) {
        double distance = std::fabs(valid_roots[i] - range_mid);
        if (distance < min_distance) {
            min_distance = distance;
            best_root = valid_roots[i];
        }
    }

    // 确保根在有效范围内
    *root = std::max(mina, std::min(maxb, best_root));
    return 1;
}

// 新的函数，处理两段曲线并计算最近点信息
// 计算曲率的辅助函数
double calculateCurvature(double ns1, double ns2, double es1, double es2) {
    double speed_sq = ns1 * ns1 + es1 * es1;
    if (speed_sq < 1e-10) return 0.0;
    return std::abs(ns1 * es2 - es1 * ns2) / std::pow(speed_sq, 1.5);
}

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
        double *heading, double *curvature, double *distance)
{
    double ns1, es1, ns1_1, es1_1, ns1_2, es1_2, ns1_3, es1_3, sk_curve1;
    double ns2, es2, ns2_1, es2_1, ns2_2, es2_2, ns2_3, es2_3, sk_curve2;

    // 计算第一段曲线上的最近点
    double dist1 = getcurvdis(skk1, skk2, vehicle_x, vehicle_y,
                              &ns1, &es1, &ns1_1, &es1_1, &ns1_2, &es1_2, &ns1_3, &es1_3, sk_curve1,
                              AAn1, BBn1, CCn1, DDn1, AAe1, BBe1, CCe1, DDe1);

    // 计算第二段曲线上的最近点
    double dist2 = getcurvdis(skk2, skk3, vehicle_x, vehicle_y,
                              &ns2, &es2, &ns2_1, &es2_1, &ns2_2, &es2_2, &ns2_3, &es2_3, sk_curve2,
                              AAn2, BBn2, CCn2, DDn2, AAe2, BBe2, CCe2, DDe2);

    // 选择距离更近的点
    if (dist1 >= 0 && dist2 >= 0) {
        if (dist1 <= dist2) {
            ref_n = &ns1;
            ref_e = &es1;
            double head = atan2(es1_1, ns1_1);
            heading = &head;  // 航向角
            double curva = calculateCurvature(ns1_1, ns1_2, es1_1, es1_2);  // 曲率
            curvature = &curva;
            distance = &dist1;
            return dist1;
        } else {
            ref_n = &ns2;
            ref_e = &es2;
            double head = atan2(es1_1, ns1_1);
            heading = &head;  // 航向角
            double curva = calculateCurvature(ns1_1, ns1_2, es1_1, es1_2);  // 曲率
            curvature = &curva;
            distance = &dist2;
            return dist2;
        }
    } else if (dist1 >= 0) {
        ref_n = &ns1;
        ref_e = &es1;
        double head = atan2(es1_1, ns1_1);
        heading = &head;  // 航向角
        double curva = calculateCurvature(ns1_1, ns1_2, es1_1, es1_2);  // 曲率
        curvature = &curva;
        distance = &dist1;
        return dist1;
    } else if (dist2 >= 0) {
        ref_n = &ns2;
        ref_e = &es2;
        double head = atan2(es1_1, ns1_1);
        heading = &head;  // 航向角
        double curva = calculateCurvature(ns1_1, ns1_2, es1_1, es1_2);  // 曲率
        curvature = &curva;
        distance = &dist2;
        return dist2;
    } else {
        return -1;  // 两段曲线都没有找到有效点
    }
    LOGI("curx:%f, cury:%f, An1:%f, Bn1:%f, Cn1:%f, Dn1:%f, Ae1:%f, Be1:%f, Ce1:%f, De1:%f, sk1:%f, sk2:%f,"// 第一段曲线参数
         "An2:%f, Bn2:%f, Cn2:%f, Dn2:%f, Ae2:%f, Be2:%f, Ce2:%f, De2:%f, sk3:%f,&ref_n:%f, &ref_e:%f, &heading:%f, &curvature:%f, &distance:%f\n", // 第二段曲线参数
         vehicle_x, vehicle_y, An1, Bn1, Cn1, Dn1, Ae1, Be1, Ce1, De1, sk1, sk2,// 第一段曲线参数
         An2, Bn2, Cn2, Dn2, Ae2, Be2, Ce2, De2, sk3,                // 第二段曲线参数
         *ref_n, *ref_e, *heading, *curvature, *distance);
}
