
// Created by lzd on 2025/05/20.     Revised by lzd on 2025/07/10.

/* Analyzed by lzd on 2025/07/11    被动式农具导航
 系统认识：1.农机与农具之间的映射系数与农具横偏、农具航偏关联    --- 入线阶段(dpsi) + 在线阶段(errImp)
           (1)农具航偏越接近零，农具与农机之间映射系数越接近于1(没有侧滑干扰)
           (2)农具横偏与映射系数呈现非线性映射关系: 小偏差阶段 侧滑干扰量与农具横偏量接近 为防止超调 微偏微调+估计补偿
 控制难点：1.农具位置调整响应农机位置的移动量存在时间滞后，且滞后时间未知可变(且变化区间较大);
         2.农机与农具之间柔性连接，农机位置调整量映射至农具位置响应量未知可变(且变化区间较大);
         3.农具极易出现向内/向外侧滑，是否发生侧滑及侧滑量多少、是否将发生侧滑及将发生的侧滑量未知可变(且变化区间较大);
         4.滞后时间、映射系数、位置侧滑受到地况、车况、土质、作业类型等诸多因素影响，极大地限制了关键参数的鲁棒性能
         5.农机与农具系统存在的高频干扰，且农机与农具模型误差较大，观测器难以准确估计干扰，无法有效实施补偿。
         6.倒车入线：倒车期间农具处于抬起状态，容易左右晃动，尤其在坡地情形下容易发散失稳
         7.前进入线：倒车精度有限，农具抬起放下位置偏差等，起步出现0~15cm的农具横偏，如何平滑、快速且不超调地入线
         8.农机可能控不住:农机侧滑、方向盘(实际转角跟不上期望转角)
           电机内部速度模式响应速度 50ms 如何实现柔和且快速转向 降低方向盘时滞影响 提高算法鲁棒性(大阻力)
           方向盘--电机转角跟踪控制 ---> 神经网络调参(数据回传 服务器调参) 神经网络智能控制 ---> 数据驱动
           期望前轮转角--农机导航路径跟踪控制 ---> 神经网络调参(数据回传 服务器调参) 神经网络智能控制 ---> 数据驱动
 控制方案：农机移线法   ✔
    性能指标：农机航偏、农具横偏、农具航偏
    控制输入：最少2个：农机航偏、农具横偏(系统输出)
    控制输出：前轮转角(系统输入)
  一、基于农机具系统整体设计方案 将求解的前轮转角传递至农机导航系统作用于方向盘  ×
    系统输入输出个数 ---> 经典控制理论  ×
    农具响应时滞(变且大)、农具响应系数(农机与农具柔性连接)、农具侧滑 ---> 数学模型误差过大 ---> 现代控制理论  ×
    目前农机导航路径跟踪控制，未能基于(神经网络)智能控制实现数据驱动 ---> 可靠性及鲁棒性太差 ---> 智能控制理论 ×
  二、基于农机系统拆分设计移线方案，基于农具当前及历史信息解算农机偏差((err + err_eso))传递至农机系统，解算前轮转角作用方向盘。 ✔
    被动式农具导航本质即为改变农机位置，进而调节农具位置，最终实现农具作业在目标导航线
    农机移线法优点：1.可以快速拓展至铰接式、履带式、四轮驱动等多种拖拉机车型，算法拓展性更强  经验设计，方案通用
                 2.依托于农机导航系统，借力当前成熟的农机导航路径跟踪算法，算法可靠性更佳  控制系统内部稳定
                 3.确定隐形的关键变量，调参实现更优跟踪效果(采集优良数据)，降低(神经网络)智能调参及控制的设计难度
                 算法鲁棒性：(1)智能调节关键变量 (2)设计可在线微调的(神经网络)智能控制实现数据驱动迭代(越跑越好)
    农机移线法局限：1.农机(农具)航向小偏差:移线方案的应用前提，此时农机的移动距离和农具的移动距离才是较为稳定的映射关系。
                   农机航向小偏差与其他场景下，农机与农具的相对位置差异很大;最终稳定的在线阶段，农机航向必定是小偏差;
                 2.农具位置大偏差场景，较长的回线期间可能出现不同程度的侧滑，农机与农具的最终相对位置将受到侧滑影响。
 控制策略：农机移线模式  解算并移动农机目标导航线
  一、经典/现代控制理论
    1.滞后时间与映射系数为给定的预估定值(数值保守)，基于农具当前及历史状态信息分析并处理农具侧滑，解算农具横偏((err + err_eso))。
    2.农机与农具无侧滑时，农具横偏先小幅增大，随后快速减小，农机与农具相对位置先增大后减小
    3.移动距离、移动周期、移动力度  策略切换:多种tricks之间如何切换+多种判定条件如何切换处理策略
    4.微偏微调(car_flag across_flag) + 小偏轻调(shift_flag slip_in_flag slip_out_flag)
    5.Tricks判据：(1)农具横偏量(航偏量)      (2)农具横偏量变化趋势           (3)农机与农具周期内各自移动量
                 (4)农机与农具相对位置      (5)当前农具与2s前农机相对位置    (6)农机与农具相对位置变化趋势
    6.移动距离(映射系数): 0~1.5cm---0.2      1.5~2.5cm---0.35      >2.5cm---0.5
      移动力度(侧滑修正): 阶段一:传入errImp; 阶段二:按照err_n超调1cm解算delta,回正阶段(dpsi*delta<0)传入err_n;
      移动周期(滞后时间): (err + err_eso) < 1cm 开启计数 (1m/s 目标计数 30) 弃用计数统计(避免修正期间侧滑干扰) (外侧滑计数)
      关键变量：映射系数、滞后时间、给定农机超调量   移线距离、力度、周期
      15cm --- 5cm --- 2.5cm --- 1.5cm --- 0.6cm
    7.前进入线、倒车入线：平滑、快速且不超调入线  ---> 农机航偏(重点)、农具航偏 + 农具横偏  ---> 舍去以往农机横偏判据
      (1)农机航偏 < 0.02rad(1.146°) 农机摆正 ---> 农机移线法(结合农具航向处理)  农机与农具需要稳定(2s)
         农具横偏变化趋势/农机与农具变化趋势/农机与农具相对位置  是否接入移线法的侧滑判定(多大农具横偏可接入)
      (2)农机航偏 > 0.02rad(1.146°) 农机斜放 ---> 农机导航适当超调，加速回正  5cm-1.5cm 10cm-3cm 15cm-4.5cm
         保留防内侧滑致使超调的策略   首次入线进度  err + overshoot_go   errImp + 1.5*dimpsi + overshoot_go
    8.农具抬起阶段控制方案切换、农具抬起判定策略
    改进思路：
        1.有限状态机实现tricks策略之间的切换
        2.不损失控制效果前提下，尽可能削弱对映射系数、滞后时间、侧滑量的估计依赖。

  二、智能控制理论
    1.智能调参：数据回传至服务器 ---> 通过(神经网络)离线调节映射系数、滞后时间、侧滑量相关参数 ---> 下发参数
    2.智能控制：经验调参/智能调参 ---> 大量优质数据(地况差、车况差，跟踪效果优) ---> 训练网络 数据驱动
    基础：基于经典/现代控制理论(经验 + 思路：smith预估、ESO补偿)设计的农机移线方法，可以实现优良的农具跟踪效果
*/

// 隐藏算法库内的函数名
#ifndef Symbol_EXPORTS
#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define Symbol_EXPORTS __declspec(dllexport)
#elif defined __GNUC__ && __GNUC__ >= 4 || defined(__APPLE__) || defined(__clang__)
#define Symbol_EXPORTS __attribute__((visibility("default")))
#endif
#endif

#include "Agguide.h"
#include "AgguideIm.h"
#include "VehicleNavControl.h"
#include "android_log.h"
#include <sys/stat.h>
#include <unistd.h>
#include "Car_ratio.h"  // 农机导航控制算法(直线+曲线)
#include "Imp_ratio.h"
#include "Imp_delta.h"
// #include "curve_point.h"

Symbol_EXPORTS void LADRCcontrolForImplement(const VehicleNavParasIn_T &data1, const VehicleNavDatasIn_T &data2, VehicleNavOut_T* data3) {
    LOGI("This is the controllers of a passive tractor-towed implement:start");   // 被动式农具导航跟踪控制算法基本框架：农机超调移线法
    double l1 = data1.L1;   // 前后轴距离
    double l2 = data1.L6;   // 拖拉机后轴到挂载点距离
    double l3 = data1.L4;   // 挂载点到犁轴距离

    // 导航线参数及其目标航向角
    double Aa = data2.LineA;
    double Bb = data2.LineB;
    double Cc = data2.LineC;
    double T_psi = atan2(Aa, -Bb);

    // 处理软件刚打开时，参数异常下发问题
    double curx, cury, impx, impy, cur_psi, imp_psi, curv, impv, rol, imp_rol;
    if ((Aa == 0 && Bb == 0) || (abs(data2.Yaw) > 10) || (abs(data2.Yaw_impmlement) > 10)) {
        curx = 0;       cury = 0;   impx = 0;   impy = 0;   cur_psi = 0;
        imp_psi = 0;    curv = 0;   impv = 0;   rol = 0;    imp_rol = 0;
    } else {
        // 软件刚打开时系统崩溃的问题在这，刚进系统时农具的初始值未下发
        cur_psi = LQR_car::deg_trans(data2.Yaw); imp_psi = LQR_car::deg_trans(data2.Yaw_impmlement);
        curx = data2.E;                 cury = data2.N;
        impx = data2.E_implement;       impy = data2.N_implement;
        curv = data2.Speed_true;        impv = data2.Speed_imp;
        rol = data2.Roll;               imp_rol = data2.Roll_implement;
        /*LOGI("curx:%f, cury:%f,curv:%f,cur_Yaw:%f,rol:%f", curx, cury, curv, cur_psi, rol);
        LOGI("impx:%f, impy:%f,impv:%f,Yawimp:%f,T_psi:%f,imp_rol:%f", impx, impy, impv, imp_psi,T_psi, imp_rol);*/
    }

    // 定义农机具系统的若干状态变量   求解农机与农具的横向误差与航向误差
    double err = 0, errImp = 0, dpsi = 0, dimpsi = 0;
    // 计算航向偏差，一个单位圆形成两个角，选择数值小于180°(M_PI)的角; cur_psi = 3.01,T_psi =-2.7 航向偏差应减去2pi
    dpsi = T_psi - cur_psi;
    if (dpsi > M_PI) { dpsi = dpsi - 2 * M_PI; }
    if (dpsi < -M_PI) { dpsi = dpsi + 2 * M_PI; }
    dimpsi = T_psi - imp_psi;
    if (dimpsi > M_PI) { dimpsi = dimpsi - 2 * M_PI; }
    if (dimpsi < -M_PI) { dimpsi = dimpsi + 2 * M_PI; }
    // 解算农机与农具横偏，处理软件刚打开时，参数异常下发问题
    if (Aa == 0 && Bb == 0) {
        err = 0;    errImp = 0;
    }
    else {
        err = (Aa * curx + Bb * cury + Cc) / sqrt(pow(Aa, 2) + pow(Bb, 2));
        errImp = (Aa * impx + Bb * impy + Cc) / sqrt(pow(Aa, 2) + pow(Bb, 2));
    }
    LOGI("Raw dpsi:%f,dimpsi:%f,err:%f,errImp:%f",dpsi, dimpsi, err, errImp);
    double err_eso = 0;
    static double delta = 0;    static double delta_ago = 0;
    err_eso = Imp_ESO(curx, cury, impx, impy, cur_psi, imp_psi, curv, impv, rol, imp_rol,
                      Pitch, PitchIm,data2.Omiga, data2.WAS, 22 , &delta, &delta_ago);
    data3->DeltaAm = delta;
    delta_ago = delta;
    LOGI("delta_ago:%f,delta:%f",delta_ago,delta);

    int on_line = 0;        int Control_Type = 0;       int time_on = 0;
    on_line = OnlineAggresivenessIm / 100 % 10;       // 默认数值65
    Control_Type = GainLearnIm / 100000 % 10;             // 默认值 25
    time_on = PTimeIm / 1000 % 10;                     // 默认值 20
    if(on_line == 1){
        data3->xTrack = err;
        data3->xTrack_implement = errImp;
    }
    if(on_line == 2){              // 仅测试使用  待定删除
        data3->xTrack = errImp;
        data3->xTrack_implement = err + err_eso;
    }
    if(on_line == 0 || on_line == 3 || on_line == 5){
        data3->xTrack = err + err_eso;
        if (err + err_eso == 0) {
            data3->xTrack = errImp * 0.65;
        }
        if (Control_Type == 2) {
            data3->xTrack = err;
        }
        data3->xTrack_implement = errImp;
    }
    if(curv < 0 && time_on == 2){
        data3->xTrack = err;
        data3->xTrack_implement = err;
    }
    data3->xHeading = dpsi;
    data3->xHeading_implement = dimpsi;
    LOGI("data3->xTrack:%f,data3->xTrack_implement:%f",data3->xTrack,data3->xTrack_implement);
    /*double curve_dist = 0,ref_n = 0,ref_e = 0,heading = 0,curvature = 0, distance = 0;
    curve_dist = Cal_Curve_Point(curx, cury, An1, Bn1, Cn1, Dn1, Ae1, Be1, Ce1, De1, sk1, sk2,// 第一段曲线参数
            An2, Bn2, Cn2, Dn2, Ae2, Be2, Ce2, De2, sk3,                // 第二段曲线参数
            &ref_n, &ref_e, &heading, &curvature, &distance);      // 输出参数*/
}