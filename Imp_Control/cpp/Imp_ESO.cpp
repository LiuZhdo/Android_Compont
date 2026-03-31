
// Created by lzd on 2025/05/20.     Revised by lzd on 2025/07/10.

/* Analyzed by lzd on 2025/07/11    被动式农具导航

   Inner Test by lzd on 2025/10/20

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
  二、基于农机系统拆分设计移线方案，基于农具当前及历史信息解算农机偏差(err_n)传递至农机系统，解算前轮转角作用方向盘。 ✔
    被动式农具导航本质即为改变农机位置，进而调节农具位置，最终实现农具作业在目标导航线
    农机移线法优点：1.可以快速拓展至铰接式、履带式、四轮驱动等多种拖拉机车型，算法拓展性更强  经验设计，方案通用
                 2.依托于农机导航系统，借力当前成熟的农机导航路径跟踪算法，算法可靠性更佳  控制系统内部稳定
                 3.确定隐形的关键变量，调参实现更优跟踪效果(采集优良数据)，降低(神经网络)智能调参及控制的设计难度
                 算法鲁棒性：(1)智能调节关键变量 (2)设计可在线微调的(神经网络)智能控制实现数据驱动迭代(越跑越好)
    农机移线法局限：1.农机(农具)航向小偏差:移线方案的应用前提，此时农机的移动距离和农具的移动距离才是较为稳定的映射关系。
                   农机航向小偏差与其他场景下，农机与农具的相对位置差异很大;最终稳定的在线阶段，农机航向必定是小偏差;
                 2.农具位置大偏差场景，较长的回线期间可能出现不同程度的侧滑，农机与农具的最终相对位置将受到侧滑影响。
  三、入线策略:
  1.农机导航入线
    优点：控制农机,稳定性高;无需重新定目标线(大偏差);不受农具是否放下影响
    缺点：坡地、农具(农机)中心偏移等导致在线阶段时农机与农具相对位置超过5cm时;
         1)切换农机移线法时机与条件难以确定;
         2)可能出现一直在控制农机，无法进入农机移线阶段；
    适用场景:在线时农机与农具的相对位置不超过3cm:小坡地/平地 且农具(农机)中心偏移输入正确 且农具中心偏移小于5cm
  2.农机移线入线
    优点:1)控制农机,稳定性较好，存在中心偏移时，仍然能够完成入线;
        2)不受农具是否放下影响；
        3)诸多策略设计，不易超调;
    缺点：1)必须依据农机具状态确定目标导航线，这依赖于农机车身摆正(农机航向小);
         2)大偏差入线阶段,诸多策略无法生效，控制精度有限
         3)入线必须分为两阶段:农机入线 + 农具入线  入线速度慢且方向盘转动特点用户难以接受 尤其是在线时农机与农具的相对位置超过5cm
  3.农机具导航入线
    优点:1)控制农具,实时性高,入线速度很快；
        2)无需定目标导航线;
        3)不控制农机,入线不分阶段,方向盘转动特点更易于用户接受;
    缺点:1)受到农具是否放下影响，三点悬挂尤为明显，农具放下时模型严重失真;
        2)容易出现超调,小偏差入线更为明显;
        3)切换在线阶段时的农机移线法时机不易把握，存在假稳态干扰;
  综上,入线策略如下：保留入线至在线阶段的切换策略
  1.农机导航 + 农机移线法          可调
    航偏不满足要求时 --- 农机导航;  航偏满足要求时 --- 农机移线法
  2.农机具模型入线 + 农机移线法     默认
    航偏 > 2.5° || 横偏 > 25cm 农机具模型入线(缓)   结合实际作业 此时农具必然抬起;
    航偏 < 2.5° && 横偏 < 25cm 农机移线法入线   结合实际作业 此时农具放下(25cm横偏应对单点悬挂)
    农机具模型可调参数 农机航偏权重 车速参数自适应

 控制策略：农机移线模式  解算并移动农机目标导航线
  一、经典/现代控制理论:用规则书写方案       应对怎样的场景，采用哪些数据，经过怎样的条件，如何处理数据，解决什么问题。
    1.滞后时间与映射系数为给定的预估定值(数值保守)，基于农具当前及历史状态信息分析并处理农具侧滑，解算农具横偏(err_n)。
    2.农机与农具无侧滑时，农具横偏先小幅增大，随后快速减小，农机与农具相对位置先增大后减小
    3.移动距离、移动周期、移动力度  策略切换:多种tricks之间如何切换+多种判定条件如何切换处理策略
    4.微偏微调(car_flag across_flag) + 小偏轻调(shift_flag slip_in_flag slip_out_flag)
    5.Tricks判据：(1)农具横偏量(航偏量)      (2)农具横偏量变化趋势           (3)农机与农具周期内各自移动量
                 (4)农机与农具相对位置      (5)当前农具与2s前农机相对位置    (6)农机与农具相对位置变化趋势
    6.移动距离(映射系数): 0~1.5cm---0.2      1.5~2.5cm---0.35      >2.5cm---0.5
      移动力度(侧滑修正): 阶段一:传入errImp; 阶段二:按照err_n超调1cm解算delta,回正阶段(dpsi*delta<0)传入err_n;
      移动周期(滞后时间): err_n < 1cm 开启计数 (1m/s 目标计数 30) 弃用计数统计(避免修正期间侧滑干扰) (外侧滑计数)
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

    车速、单点/三点悬挂、农具横偏参数自适应设计
    // 农具类型 + 车速 参数自适应 : 1.自定义单一值(调试) 2.默认参数值 自适应车速及农具 3.自定义 自适应车速及农具
    delay     农具响应滞后时间
    err_pre 农机已移动量对农具的影响
    pre_shoot overshoot  go_overshoot 农机超调量响应
    shift_R1 shift_R2  映射系数 农机调整量映射至农具调整量
    q1 q2 r  q00 q22 r00  控制量权重系数

    2025年10月18日 三点悬挂特性
    1.农具响应滞后时间相较于单点悬挂更短，预留于农机的调整时间更短;
    2.执行器时滞 下发目标转角后,方向盘旋转到目标位置需要时间,车身航向调整到期望位置需要时间;
      具体表现:农机往往会冲出一段距离后回线;但由于三点悬挂特性导致预留于农机调整的时间较短，常会导致农具被带出;
      地矿良好时,图像表现为小凸包--小s弯,地矿差时，图像表现为大凸包--大s弯

    在线阶段abs(err_n) <= 1cm && abs(errImp) <= 2cm 农机航向应当更大权重 农机航向必须更稳定  具体原因在于每次车身航向的调整都需要较长时间 农机容易出现冲出
    农具横偏修正阶段 应当在农机接近入线时 增加航向权重 加快农机航向回正

    1.农具侧滑，偏离导航线后，方能调整农机位置间接移动农具；
    2.调整农机位置后，需要间隔一段时间，农具方能响应。
    常规导航控制农机行成小/大肚弯，体现在农具导航极容易形成大/小S弯
    目前有效解决办法是降低移动量延长移动周期，更接近常规导航。农具导航与常规导航取折衷
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
#include "android_log.h"
#include <sys/stat.h>
#include <unistd.h>
#include "Car_ratio.h"  // 农机导航控制算法(直线+曲线)
#include "Imp_ratio.h"
#include "Imp_delta.h"
#include "KYLOG_C_APIs.h"
#define ago_m 20        // 用于存储2s/3s内农机与农具历史数据信息    待定

// 日志存储: 1.力矩99三份最新日志循环存储 2.日志数据加密--编号+数据+表头 3.日志内容及顺序 度数 厘米
// 1.检查删掉日志文件，不重启软件下，是否会重新创建文本(编写代码满足删除后不开软件仍能创建文本的需求)  2.硬地校准日志持续刷新持续存储
// 服务器日志上传，外测场景实车数据用于神经网络训练学习(耗费流量)    表头--选择性粘贴跳过空单元--交叉粘贴隐藏数据标签     待定
static int save_RQ_flag = 1;           // 用于标记存储第几份日志
static int save_RQ_flag_ago = 1;       // 用于清空待存文件的往期日志
long file_size_now = 0;             // 日志文件当前时刻的文件大小
FILE* fpControl = nullptr;
char impControl[1024] = "/sdcard/ControllerX/Imp_Control.txt";
char impControl_1[1024] = "/sdcard/ControllerX/Imp_Control_1.txt";
char impControl_2[1024] = "/sdcard/ControllerX/Imp_Control_2.txt";

// 检测日志存储量 <= 50M 超出设定存储量--清除已存放日志
void CheckFileOver( FILE *fp, int FileSizeMB ) {
    int s_log_file_fd = -1;
    struct stat logFileStat = {0};
    s_log_file_fd = fileno(fp);
    fstat(s_log_file_fd,&logFileStat);   // logFileStat.st_size 类型lld 单位字节B %ld  %d均无法正常输出数据 计数精度4KB, 不足4KB部分舍去
    //if ( ( ( logFileStat.st_size * 1.0 ) / ( 1024 * 1024 ) ) > FileSizeMB && (fabs(MachineSpeed*0.01) <= 0.1 || DriverStatus == 1)) // 精度丢失 窄化转换 发出警告
    if (((logFileStat.st_size) / (1024 * 1024)) >= FileSizeMB &&
        (fabs(MachineSpeed * 0.01) <= 0.1 || DriverStatus == 1))   // > 1M / >= 2M
    {
        save_RQ_flag++;
        if (save_RQ_flag <= save_RQ_flag_ago) {
            ftruncate(s_log_file_fd, 0);
            lseek(s_log_file_fd, 0, SEEK_SET);
        }
        fpControl = nullptr;
    }
    file_size_now = logFileStat.st_size;
}

Symbol_EXPORTS double Imp_ESO(double curx, double cury, double impx, double impy, double cur_psi, double imp_psi, double curv, double impv, double rol,
                              double imp_rol, double pitch, double imp_pitch, double Omiga, double WAS, int imp_mode,double *delta, double *RQ_ago) {
    LOGI("This is the controllers of a passive tractor-towed implement:start");   //农机超调移线法
    kylog_init("Imp_RQ_Control","/sdcard/ControllerX/",50*1024*1024,3, false);
    static long file_size_cmt = 0;     // 日志大小若长期为0, 则fpControl重新初始化NULL  与单次存储日志大小相关
    double mini_vel_limit = VehicleSpeedMin / 36.0;

    // 农具模型参数  l2影响调整阶段农具先增大的幅度与持续时间  l2与l3影响农具响应的滞后时间
    // 目前新方法仅涉及l1  l2与l3的影响是否引入以及如何引入，后期搭建神经网络控制器值得引入。     待定
    double l1 = WheelBase * 1.0 / 100;   // 前后轴距离
    double l2 = LeverTRT * 1.0 / 100;   // 拖拉机后轴到挂载点距离
    double l3 = LeverTIR * 1.0 / 100;   // 挂载点到犁轴距离
    LOGI("Imp Para L1 %f L2 %f L3 %f ", l1, l2, l3);

    // 导航线参数及其目标航向角
    double Aa = LineA;
    double Bb = LineB;
    double Cc = LineC;
    double T_psi = atan2(Aa, -Bb);
    LOGI("Navigation Line A %f B %f C %f T_psi %f", Aa, Bb, Cc, T_psi);

    if(imp_mode != 22){
        // 处理软件刚打开时，参数异常下发问题
        if ((Aa == 0 && Bb == 0) || (abs(cur_psi) > 10) || (abs(imp_psi) > 10)) {
            curx = 0;       cury = 0;   impx = 0;   impy = 0;   cur_psi = 0;
            imp_psi = 0;    curv = 0;   impv = 0;   rol = 0;    imp_rol = 0;
        } else {
            // 软件刚打开时系统崩溃的问题在这，刚进系统时农具的初始值未下发
            cur_psi = LQR_car::deg_trans(cur_psi); imp_psi = LQR_car::deg_trans(imp_psi);
        }
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

    // 农机具系统的历史状态信息  tricks策略依赖数据 ---> 解算err_n ---> 传入农机路径跟踪算法
    // 关键策略: 农机近1s移动量 预测农具未来遇到趋势 提前应对     隔数点采样判别农具横偏变化趋势
    int err_pre = 10;
    int down_pos = 2;                   static double dimer[4];
    static int ago_i = ago_m -1;        static double dpsi_ago[ago_m];  // 历史数据计数
    static double err_ago[ago_m];       static double errImpago[ago_m]; // 农机横偏量   农具横偏量

    // 农机具系统的当前状态及上一时刻状态  农机路径跟踪算法依赖数据 ---> 解算delta
    Car_State cur_state = {0, 0, 0, 0, 0, 0, 0};                  // 用于存储当前农机状态信息
    Imp_State imp_state = {0, 0, 0, 0, 0, 0, 0};                  // 用于存储当前农具状态信息
    Car_State cur_ago = {0, 0, 0, 0, 0, 0, 0};                    // 用于存储上一刻农机状态信息
    Imp_State imp_ago = {0, 0, 0, 0, 0, 0, 0};                    // 用于存储上一刻农具状态信息
    // 保存当前农机具系统状态变量
    cur_state = {curx, cury, cur_psi, curv, rol, dpsi, err};
    imp_state = {impx, impy, imp_psi, impv, imp_rol, dimpsi, errImp};
    // control tricks的过程状态变量
    // 控制方案切换   初次入线  农机移线法启用
    int Control_Type = 0;           int delay = 0;
    // 关键参数: 单次调节周期 --- 农具响应滞后时间(delay)  农机超调量(overshoot)  映射系数(0.2  0.35  0.5)
    static double pre_shoot = 0;    static int over_flag = 0;       // 控制阶段切换词(农具横偏两阶段修正)
    static double overshoot = 0;    static double go_overshoot = 0; // 农机超调量
    // 经过处理后输入农机导航系统用于求解前轮转角的修正横偏
    static double err_n = 0;        static double errn_std = 0;       static double errn_std_ago = 0;

    // 农机导航控制量与控制律
    static double err_eso = 0;      Eigen::Vector2d car_state;
    double delta_LQR = 0;   static Eigen::MatrixXd K_car(2, 2);
    double q1 = 0;          double q2 = 0;          double r = 0;
    double car_K1 = 0;      double car_K2 = 0;      double car_U = 0;
    double delta_imp = 0;   static Eigen::MatrixXd K_imp(2, 3);
    double qq0 = 0;         double qq2 = 0;         double rr0 = 0;
    double imp_K0 = 0,imp_K1 = 0,imp_K2 = 0;        double imp_U0 = 0;
    double dpsi_ratio = 0;  double dpsi_back = 0;   double imp_max = 0;

    // 求解修正横偏(err_n)所需的标志量及其相关计数 各种tricks标志词
    // 农机移线标志词+计数                        shift_flag
    int shift_cnt = 0;                  int ago_L = 10;                   // 用于计算的历史数据长度
    double shift_value = 0;             double shift_dis = 0;             // 启用移线判据的临界横偏及移线距离
    static double shift_std = 0;        static double shift_std_ago = 0;  // 移线周期的目标导航线
    static int shift_cout = 0;          static int target_cout = 0;       // 移线周期及计数
    double shift_R = 0;                 double shift_f = 0;               // 映射系数+移线周期系数
    double shift_R1 = 0;                double shift_R2 = 0;
    double min_val = 0;                 double imp_mean = 0.0;            // 农具横偏均值滤波
    double imp_ten_mean = 0.0;          double imp_ten_sum = 0.0;
    double imp_five_mean = 0.0;         double imp_five_sum = 0.0;

    // 特殊情况处理策略标志词               car_flag across_flag
    static int car_cmt = 0;             static int five_flag = 0;
    static int car_online = 0;          static int car_shift = 0; // (农机回正)标志词
    static int car_flag = 0;            static int car_flag_ago = 0;      // 移线阶段标志词
    static double err_std = 0;          static double errImp_std = 0;     // 移线周期的起始农机及农具横偏
    static int across_flag = 0;         static int shift_flag = 0;
    static int slip_out_flag = 0;       static int slip_in_flag = 0;
    static int slip_out_trick = 0;      static int slip_in_trick = 0;

    // 应对快速侧滑：修正期间横偏仍继续增大或快速减小    slip_in_flag  slip_out_flag
    static double errImp_out = 0;       // 农具大侧滑标志量，用于防侧滑
    static int Imp_out_cnt = 0;         double errImp_dis = 0;      // 农具的移动量 外侧滑解算err_n的农具计数
    static int Imp_in_flag = 0;         double cross_in_num = 0;    // 内侧滑解算标志词
    int slip_in_std = 0;                static double car_state_ago = 0;
    static int slip_out_cmt = 0;        static int slip_out_cnt = 0;
    static double max_pos = 0;          static int max_pos_flag = 0;

    // 大偏差前进入线
    static int off_shift_std = 0;       double go_ratio = 0;           // 大偏差农机相对位置、映射系数
    double go_over_max = 0;             double go_over_ratio = 0;      // 大偏差入线超调量、超调系数(求超调量)、超调上限 入线加速
    static double go_cross = 0;         static int go_cross_flag = 0;
    static int go_cnt = 0;              static int go_cnt_last = 0;
    static int go_flag = 0;             int go_stay_cnt = 0;
    double go_imp_sum = 0;              double go_imp_mean = 0;
    double errImp_max = 0.55;           double errImp_upper = 0;        // 关键策略 限制农机最大航偏入线

    // 倒车入线
    static int back_cnt = 0;
    static int back_cmt = 0;            int back_stay_cnt = 0;
    double back_imp_sum = 0;            double back_imp_mean = 0;

    // 农具控制模式： 22 / 13+19(GainSteeringIm_mode)
    // 农具导航类型: 学习灵敏度百位数:0 农机移线法 2 农机导航;  
    // 农机导航算法: 转向灵敏度百位数:农机导航 13+19 Imp_RQ(默认) 22 Imp_delta 入线策略;
    int GainSteeringIm_mode = 35, GainxHeadingIm_mode = 27, GainxTrackIm_mode = 100;
    int GainRIm_mode = 80, OnlineAggresivenessIm_mode = 60, ApproachAggresivenessIm_mode = 70;
    // 模式13参数调整
    if(imp_mode != 22){
        // GainxHeadingIm % 10 == 1 映射系数采用默认值
        GainSteeringIm_mode = ApproachAggresivenessIm - 35;    // 转向灵敏度由入线进度(70)传入 默认值35
        if(ApproachAggresivenessIm >=60 && ApproachAggresivenessIm <= 70 || ApproachAggresivenessIm == 80){
            GainSteeringIm_mode = 35;
        }
        GainRIm_mode = GainR + 25;
        if(GainxTrackIm % 5 != 1){
            OnlineAggresivenessIm_mode = GainxHeadingIm;      // 在线进度由航向灵敏度传入
            ApproachAggresivenessIm_mode = GainxTrackIm;      // 入线进度由横向灵敏度传入
        }
        else{
            OnlineAggresivenessIm_mode = 55;
            ApproachAggresivenessIm_mode = 65;
        }
    }
    if(imp_mode == 22){
        GainRIm_mode = GainRIm;
        GainSteeringIm_mode = GainSteeringIm;
        GainxTrackIm_mode = GainxTrackIm;
        GainxHeadingIm_mode = GainxHeadingIm - GainxHeadingIm % 5;
        ApproachAggresivenessIm_mode = ApproachAggresivenessIm;
        OnlineAggresivenessIm_mode = OnlineAggresivenessIm;
    }
    int steer = 0;      int heading = 0;      int xtrack = 0;       int gain_r = 0;
    int on_line = 0;    int go_line = 0;      int time_on = 0;      int time_off = 0;
    // 导航类型 学习灵敏度百位数: 0.农机移线法(默认); 1.智能控制算法; 2.农机导航(农机能否跑直线); 个十位数: (小偏差)移线周期标定值(30)
    Control_Type = GainLearnIm / 100000 % 10;          // 默认值 25
    // 农机控制算法类型  转向灵敏度百位数: 0/1.Car_LQR  0 车速自适应  1 单值调试 ;
    // 入线方法切换: 默认方法：个位数为1则调用农机导航+农机移线法(在线时农机与农具相对位置小 三点悬挂推荐)   农机具模型法 + 农具移线法(相对位置大 单点悬挂推荐)
    steer = GainSteeringIm_mode / 100 % 10;                 // 默认值 10
    // 航向灵敏度百位数: 0.自定义航向    1.自定义横向 航向 控制;   个位数 在线超调量
    heading = GainxHeadingIm_mode / 100 % 10;              // 默认数值 20
    // 个位数决定在线进度与入线进度是否用默认值( = 1)
    xtrack =  GainxTrackIm_mode / 100 % 10;                //默认数值 13/40    22/100
    // 百位数2 调节倒车
    gain_r =  GainRIm_mode / 100 % 10;                     //默认数值 13/60   22/100
    // 在线进度百位数为0，自定义1.5~2.5cm映射系数 0:err_n  1:显示农机真实横偏   2:数字为农具横偏  3:car_state(1)  5:大偏差限幅err_n
    on_line = OnlineAggresivenessIm_mode / 100 % 10;       // 默认数值55
    // 自定义2.5~5cm映射系数  个位数为1或者6 单值调试
    go_line = ApproachAggresivenessIm_mode / 100 % 10;     // 默认数值65
    // 自定义(微偏差)移线周期标定值(35)   在线判断时间百位数 倒车 0 一次性农机移线法  1 策略农机移线法 2 纯农机导航 3 农机具模型法倒车
    time_on = PTimeIm / 1000 % 10;                     // 默认值 20
    // 离线判断时间 Imp_out_cnt 百位数 农机预测长度   个位数与十位数结合为在线超调量
    time_off = PTimeOffIm / 1000 % 10;                 // 默认值 10

    // 在线阶段 中途停车未取消自动/前进途中 点击线偏移(3.5cm) 大偏差情况需要重新做首次入线处理
    if((abs(errImpago[0] - errImpago[1]) > 0.035) && abs(errImp) > 0.07 && car_online == 1){
        go_flag = 0;
        car_shift = 0;
        car_online = 0;
        five_flag = 0;
    }

    // 手动及倒车状态下赋初值  取值0.1避免溜车干扰   倒车、停车、手动状态 数据复位与数据保持
    if (DriverStatus == 0x01 || (curv < -0.1 && time_on != 1) || (abs(curv) < mini_vel_limit)) {
        // 关键变量参数 + 农机移线法启动 + 首次入线标志词(倒车必须清零)
        five_flag = 0;        delay = 0;            // 滞后时间
        car_shift = 0;        car_online = 0;
        pre_shoot = 0;        over_flag = 0;        // 控制两阶段切换标志词
        overshoot = 0;        go_overshoot = 0;     // 超调量解算

        // 农机移线法 调节参数
        err_std = 0;          errImp_std = 0;
        shift_cout = 0;       target_cout = 0;

        // 求解控制量时所需的标志量及其相关计数 各种tricks标志词
        // Car_Shift --- 农机移线 --- 目标导航线
        slip_out_trick = 0;   slip_in_trick = 0;
        car_cmt = 0;          car_flag = 0;         car_flag_ago = 0;
        across_flag = 0;      errn_std = 0;         errn_std_ago = 0;   // 移线周期交替标志词
        shift_flag = 0;       slip_out_flag = 0;    slip_in_flag = 0;

        // 应对快速侧滑： 修正期间横偏仍继续增大或快速减小
        Imp_in_flag = 0;      Imp_out_cnt = 0;      car_state_ago = 0;
        slip_out_cmt = 0;     slip_out_cnt = 0;     max_pos = 0;
        max_pos_flag = 0;     errImp_out = 0;

        // 前进入线处理
        go_cnt = 0;           go_cnt_last = 0;
        off_shift_std = 0;    go_overshoot = 0;
        go_flag = 0;          go_cross = 0;         go_cross_flag = 0;

        if(DriverStatus == 0x01 || (abs(curv) < mini_vel_limit)){          // 自动倒车期间不清零
            back_cnt = 0;   back_cmt = 0;
            *delta = 0;     car_state(0) = 0;     car_state(1) = 0;
            if(DriverStatus == 0x01){
                err_n = 0;      shift_std = 0;        shift_std_ago = 0;  // 经过处理后输入控制器的农机具系统横偏
            }
        }
    }

    // 保存农机横偏、农具横偏、农具横偏变化量、移线计数的历史数据
    if (ago_i <= -1) {
        for (int kk = ago_m - 2; kk >= 0; kk--) {
            dpsi_ago[kk + 1] = dpsi_ago[kk];
            err_ago[kk + 1] = err_ago[kk];
            errImpago[kk + 1] = errImpago[kk];
            LOGI("errImpago[%d]:%f,dpsi_ago[%d]:%f,err_ago[%d]:%f",
                 kk + 1, errImpago[kk + 1],kk + 1, dpsi_ago[kk + 1], kk + 1, err_ago[kk + 1]);
        }
        // 当前农具横偏与前2s的每个节点的农具横偏差值
        dpsi_ago[0] = dpsi;     // 此为当前值
        err_ago[0] = err;
        errImpago[0] = errImp;
        // 依次为 0-5、5-10、10-15、15-20、0-10、10-20、0-20横偏变化量
        // 正负差异性影响  dimer[0] * errImpago[9] > 0 横偏趋势增大  < 0 横偏趋势减小
        dimer[0] = errImpago[0] - errImpago[5];     // 0.5s
        dimer[1] = errImpago[0] - errImpago[9];     // 1s
        dimer[2] = errImpago[0] - errImpago[14];    // 1.5s
        dimer[3] = errImpago[0] - errImpago[19];    // 2s
        // 1.5m/s 2s    1.5~2.25m/s 1.5s    2.25~3.25m/s  1s  0.5s
        if(abs(curv) < 1){down_pos = 3;}
        if(abs(curv) >= 1 && abs(curv) < 2.25){down_pos = 2;}
        if(abs(curv) >= 2.25 && abs(curv) < 3.25){down_pos = 1;}
        if(abs(curv) >= 3.25){down_pos = 0;}
        LOGI("errImpago[0]:%f,dpsi_ago[0]:%f,err_ago[0]:%f,dimer[0]:%f,dimer[1]:%f,dimer[3]:%f,dimer[%d]:%f",
             errImpago[0],dpsi_ago[0],err_ago[0],dimer[0],dimer[1],dimer[2],down_pos,dimer[down_pos]);
    }
    if (ago_i >= 0) {
        dpsi_ago[ago_i] = dpsi;
        err_ago[ago_i] = err;
        errImpago[ago_i] = errImp;
        ago_i = ago_i - 1;
    }
    LOGI("ago_i:%d",ago_i);

    // 被动式农具导航跟踪控制算法基本框架：农机移线法
    // 自动行进期间解算前轮转角   mini_vel_limit最低车速 一般为0.7km/h 超低速为0.1km/h
    if (DriverStatus == 0x02 && fabs(curv) > mini_vel_limit) {
        if(curv < 0){
            back_cmt ++;
            if(back_cmt >= 15){
                back_cmt = 15;
            }
        }
        // 关键调试参数: delay    overshoot   shift_R/R1/R2         可调参数设置需要综合调整
        // time_off 预测: 关注前n秒农机移动量,对未来农具将会移动量影响的预测
        if (time_off < 5) {               // out_delay可取值 0 1 2 3 4 5 10 15 20
            err_pre = time_off * 5;
            if (time_off == 0) {        // 基础值为10
                err_pre = 10;           // 默认值10  此处需要调整 为0时应当为默认值
            }
            if ((ImplementType == 1 || ImplementType == 2) || GainLearnIm == 16 || GainLearnIm == 36) {         // 车速 + 悬挂类型
                if(ImplementType == 1){  // 三点悬挂
                    if(abs(curv) > 2.25 && abs(curv) <= 3.5){
                        err_pre = err_pre - 3;
                    }
                    if(abs(curv) > 3.5){
                        err_pre = err_pre - 6;
                    }
                }
                if(ImplementType == 2){  // 单点悬挂
                    if(abs(curv) < 0.8){
                        err_pre = err_pre + 3;
                    }
                    if(abs(curv) >= 0.8 && abs(curv) <= 2.25){
                        err_pre = err_pre;
                    }
                    if(abs(curv) > 2.25 && abs(curv) <= 3.25){
                        err_pre = err_pre - 2;
                    }
                    if(abs(curv) > 3.25 && abs(curv) <= 4){
                        err_pre = err_pre - 5;
                    }
                    if(abs(curv) > 4){
                        err_pre = err_pre - 7;
                    }
                }
            }
        }
        if (time_off >= 5){
            err_pre = time_off - 5;
        }
        if(err_pre >= 19){
            err_pre = 19;
        }
        LOGI("err_pre:%d,ImplementType:%d,curv:%f",err_pre,ImplementType,curv);

        if(time_on == 1 && abs(curv) >= 1.75 ){
            time_on = 0;    // 车速快倒车，农具容易摆，强制不允许移线法倒车
        }

        // 超调回线(激进)  是否加入Smith预估机制  基于农机模型做时滞补偿(预测)     待定(加入做备选)
        // 农具类型 + 车速 参数自适应 : 1.自定义单一值(调试) 2.默认参数值 自适应车速及农具 3.自定义 自适应车速及农具
        pre_shoot = GainLearnIm / 1000 % 5 * 0.0025;
        if(pre_shoot != 0){
            pre_shoot = pre_shoot - abs(curv) * 0.0005;
            if(pre_shoot <= 0){ pre_shoot = 0;}
        }
        if ((PTimeOffIm / 10 - time_off * 100) <= 35 && (ImplementType == 1 || ImplementType == 2)) {
            overshoot = (PTimeOffIm / 1000.0 - time_off) / 10.0;   // 10---1cm 小偏差预估农机超调量 调节力度 0~3cm
            if(ImplementType == 1){      // 三点悬挂
                if(abs(curv) <= 3){
                    overshoot = overshoot - abs(curv) * 0.0015;
                }
                if(abs(curv) > 3){
                    overshoot = 0.0035;
                }
            }
            if(ImplementType == 2){      // 单点悬挂
                if(abs(curv) <= 3){
                    overshoot = overshoot - abs(curv) * 0.0015 + 0.001;
                }
                if(abs(curv) > 3){
                    overshoot = 0.0045;
                }
            }
        }
        if(((PTimeOffIm / 10 - time_off * 100) > 35 && (PTimeOffIm / 10 - time_off * 100) < 50) ||
            (ImplementType != 1 && ImplementType != 2) || GainLearnIm == 16 || GainLearnIm == 36){
            if(ImplementType == 1){      // 三点悬挂
                if(abs(curv) <= 3){
                    overshoot = 0.01 - abs(curv) * 0.0015;
                }
                if(abs(curv) > 3){
                    overshoot = 0.0035;
                }
            }
            if(ImplementType == 2){      // 单点悬挂
                if(abs(curv) <= 3.5){
                    overshoot = 0.011 - abs(curv) * 0.0015;
                }
                if(abs(curv) > 3.5){
                    overshoot = 0.0045;
                }
            }
        }
        if((PTimeOffIm / 10 - time_off * 100) >= 50 && (PTimeOffIm / 10 - time_off * 100) <= 85 ){          // 单一值 不随车速、农具类型自适应
            overshoot = ((PTimeOffIm - 50) / 1000.0 - time_off) / 10;          // 自定义 60 1cm
        }
        if(imp_mode != 22){
            overshoot = overshoot - 0.001;
            if(overshoot < 0.003){
                overshoot = 0.003;
            }
        }

        if(ImplementType == 1){
            go_over_ratio = 0.135;
            if(abs(errImp) <= 0.15){
                go_over_ratio = 0.15;
            }
            if(abs(errImp) > 0.3){
                if(abs(curv) < 1.75){
                    go_over_ratio = 0.115 + 0.05 * ((PTimeIm/10) % 5);
                }
                else{
                    go_over_ratio = 0.1;
                }
            }
            go_over_max = 0.07;
            if(abs(errImp) <= 0.5){
                go_over_max = 0.065;
            }
            if(abs(errImp) > 0.65){
                go_over_max = 0.08;
            }
        }

        if(ImplementType == 2){
            go_over_ratio = 0.15;
            if(abs(errImp) <= 0.15){
                go_over_ratio = 0.185;
            }
            if(abs(errImp) > 0.3){
                if(abs(curv) < 1.75){
                    go_over_ratio = 0.135 + 0.05 * ((PTimeIm/10) % 5);
                }
                else{
                    go_over_ratio = 0.1;
                }
            }
            go_over_max = 0.08;
            if(abs(errImp) <= 0.5){
                go_over_max = 0.075;
            }
            if(abs(errImp) > 0.65){
                go_over_max = 0.09;
            }
        }
        if(ImplementType != 1 && ImplementType != 2){
            go_over_ratio = 0.125;               // 大偏差入线超调系数
            go_over_max = 0.07;
        }
        if(abs(curv) > 1.5){
            go_over_ratio = go_over_ratio - 0.1 * (abs(curv) - 1.5);
            go_over_max = go_over_max - 0.015 * (abs(curv) - 1.5);
            if(go_over_ratio < 0.045){
                go_over_ratio = 0.045;
            }
            if(go_over_max < 0.045){
                go_over_max = 0.045;
            }
        }

        LOGI("err_pre:%d,overshoot:%f, go_over_ratio:%f, go_over_max:%f, heading:%d", err_pre,
             overshoot, go_over_ratio, go_over_max, heading);

        // Smith预估机制尝试  农机模型  时滞估计    引入PI(积分)(侧滑处理已经有积分意味了)
        // 移动距离系数    在线进度:0.6~2.5cm  离线进度：映射系数 2.5cm~3.5cm
        shift_R1 = 0.5;
        if (GainxTrackIm % 10 == 1 || GainLearnIm == 16 || GainLearnIm == 36) {
            if(ImplementType == 1){   // 三点悬挂
                if(abs(curv) <= 3){
                    shift_R1 = 0.55 - abs(curv) * 0.065;
                }
                if(abs(curv) > 3){
                    shift_R1 = 0.4;
                }
            }
            if(ImplementType == 2){
                if(abs(curv) <= 3.5){
                    shift_R1 = 0.6 - abs(curv) * 0.065;
                }
                if(abs(curv) > 3.5){
                    shift_R1 = 0.45;
                }
            }
        } else {
            shift_R1 = OnlineAggresivenessIm_mode / 100.0 - on_line;
            if(ImplementType == 1){
                shift_R1 = shift_R1 - abs(curv) * 0.065;
                if(shift_R1 <= 0.3 && abs(curv) > 3){
                    shift_R1 = 0.3;
                }
            }
            if(ImplementType == 2){
                shift_R1 = shift_R1 - abs(curv) * 0.065;
                if(shift_R1 <= 0.35 && abs(curv) > 3.5){
                    shift_R1 = 0.35;
                }
            }
        }
        shift_R2 = 0.65;
        // 适配202505春播期间试用版本下的农具导航高级参数   最新软件强制刷新农具导航的高级参数 GainLearnIm == 16 || GainLearnIm == 36
        if (GainxTrackIm % 10 == 1 || GainLearnIm == 16 || GainLearnIm == 36) {
            if(ImplementType == 1){   // 三点悬挂
                if(abs(curv) <= 3){
                    shift_R2 = 0.6 - abs(curv) * 0.065;
                }
                if(abs(curv) > 3){
                    shift_R2 = 0.35;
                }
            }
            if(ImplementType == 2){
                if(abs(curv) <= 3.5){
                    shift_R2 = 0.7 - abs(curv) * 0.065;
                }
                if(abs(curv) > 3.5){
                    shift_R2 = 0.4;
                }
            }
        } else {
            shift_R2 = ApproachAggresivenessIm_mode / 100.0 - go_line;
            if(ImplementType == 1){
                shift_R2 = shift_R2 - abs(curv) * 0.065;
                if(shift_R2 <= 0.35 && abs(curv) > 3){
                    shift_R2 = 0.35;
                }
            }
            if(ImplementType == 2){
                shift_R2 = shift_R2 - abs(curv) * 0.065;
                if(shift_R2 <= 0.4 && abs(curv) > 3.5){
                    shift_R2 = 0.4;
                }
            }
        }
        // 自定义单值(调试)
        if(ApproachAggresivenessIm_mode % 5 == 1){
            shift_R1 = OnlineAggresivenessIm_mode / 100.0 - on_line;
            shift_R2 = ApproachAggresivenessIm_mode / 100.0 - go_line;
        }

        err_n = err - shift_std;    // 记录上一周期在当前时刻的err_n值(用于策略处理)
        LOGI("Last time err_n:%f", err_n);

        // 入线阶段  倒车及前进入线处理: 平滑、快速且不超调   车速自适应 + 横偏自适应 + 航偏自适应  单点悬挂、三点悬挂  待定
        if ((Control_Type != 2) && (car_online == 0)) {
            LOGI("goline shift_std_ago:%f", shift_std_ago);
            // 处理倒车入线困难问题   外测场景采集、分析数据并制定策略       待定
            if (curv < 0 && (time_on == 0)) {                        // 农机导航倒车入线
                LOGI("back tricks car_offline");
                if (back_cnt == 0 && abs(impv) > 0.15 && back_cmt>= 15) {
                    back_stay_cnt = 0; back_imp_sum = 0; back_imp_mean = 0;
                    for (int ii = 0; ii <= ago_m - 1; ii++) {                    // 入线期间定线无需关注农具横偏
                        if (abs(dpsi_ago[ii]) < 0.025) {
                            back_stay_cnt++;
                            back_imp_sum = back_imp_sum + errImpago[ii];
                            if (back_stay_cnt >= 15) {
                                back_imp_mean = back_imp_sum / back_stay_cnt;
                                shift_std = err - back_imp_mean;
                                LOGI("back shift std:%f", shift_std);
                                back_cnt = 1;
                            }
                        }
                    }
                    LOGI("back_log:back_stay_cnt:%d,back_imp_sum:%f, back_imp_mean:%f,shift_std:%f,back_cnt:%d",
                         back_stay_cnt, back_imp_sum, back_imp_mean, shift_std, back_cnt);
                }
                if (back_cnt >= 1) {
                    back_cnt++;
                    if (back_cnt >= 20) {
                        back_cnt = 0;
                    }
                }
                // LOGI("back_cnt:%d", back_cnt);
            }

            // 前进入线平滑、快速且不超调   15cm --- 5cm --- 2.5cm
            // 农机航偏小于0.02rad(1.146°)，启用农机移线法
            if (curv > mini_vel_limit) {
                LOGI("Go Line Trick.");
                // 农具横偏变化趋势 2s内横偏变化小于1.5cm: (1)起步时农机已摆正 (2)斜放 --> 摆正  启用农机移线法
                go_imp_mean = 0.0, go_imp_sum = 0.0, go_stay_cnt = 0;       // 不区分农机横偏      1.5s变化量
                // 目的：排除局部假稳态点(1~5个假稳态点)
                if (ago_i <= -1) {
                    for (int i = 0;i < ago_m;i++) {
                        if (abs(dpsi_ago[i]) < 0.032) {           // 1.8336   区分农机是否已经摆正 判定是否启用农机移线法
                            go_stay_cnt++;
                            go_imp_sum = go_imp_sum + errImpago[i];
                            int car_shift_ago = 0;
                            if (go_stay_cnt >= 10) {
                                car_shift_ago = car_shift;
                                car_shift = 1;      // 判断车身是否进入过渡稳态阶段(排查假稳态现象) + 是否启用农机移线法
                                if (car_shift == 1 && car_shift_ago == 0) {
                                    go_flag = 0;
                                }
                                go_imp_mean = go_imp_sum / go_stay_cnt;
                            }
                            /*LOGI("car_shift_ago:%d,car_shift:%d,go_stay_cnt:%d,go_flag:%d,go_imp_mean:%f",
                                 car_shift_ago, car_shift, go_stay_cnt, go_flag, go_imp_mean);*/
                        }
                    }
                }
                if (go_imp_mean == 0) {
                    go_imp_mean = errImp;
                }
                // 是否启用农机移线法    农机是否摆正
                if (abs(errImp) >= 0.05 || (abs(dpsi) >= 0.025)) {
                    off_shift_std = 0;
                    // 根据不同农具横偏量调整映射系数
                    if (abs(errImp) > 0.3) {
                        go_ratio = 1;
                    }
                    if (abs(errImp) > 0.15 && abs(errImp) <= 0.3) {
                        if(ImplementType == 1){
                            go_ratio = 0.85;
                        }
                        if(ImplementType == 2){
                            go_ratio = 0.9;
                        }
                    }
                    if (abs(errImp) >= 0.05 && abs(errImp) <= 0.15) {
                        if(ImplementType == 1){
                            go_ratio = 0.75;
                        }
                        if(ImplementType == 2){
                            go_ratio = 0.85;
                        }
                    }
                    if (abs(errImp) < 0.05) {
                        if(ImplementType == 1){
                            go_ratio = 0.35;
                        }
                        if(ImplementType == 2){
                            go_ratio = 0.45;
                        }
                    }
                    LOGI("go_imp_mean:%f,go_ratio:%f", go_imp_mean, go_ratio);
                    // 农机定线
                    if (go_flag == 0) {          // go_flag = 0 未完成定线   go_flag = 1  已完成定线   5cm || 0.025rad仅定线一次
                        if (car_shift == 1) {         // 已经启用农机农机超调移线法 标定超调量/移线量  shift_std
                            go_cnt = 0;
                            err_std = err;
                            errImp_std = errImp;
                            if (errImp * dimpsi < 0) {
                                shift_std = err - go_ratio * (go_imp_mean +
                                            (err_ago[0] - err_ago[err_pre - 1]) +
                                            1.3 * dimpsi);        // 此处是否需要引入1s的农机横偏减少量影响
                                LOGI("car_shift_1 shift std:%f", shift_std);
                            } else {
                                /*shift_std = err - go_ratio * go_imp_mean -
                                            (err_ago[0] - err_ago[err_pre - 1]) - 0.5 * dimpsi;*/
                                shift_std = err - go_ratio * (go_imp_mean +                          // 农具航偏与农机横偏变化的影响应当直接作用于农具横偏
                                            (err_ago[0] - err_ago[err_pre - 1]) + 0.5 * dimpsi);
                                LOGI("car_shift_2 shift std:%f", shift_std);
                            }
                            LOGI("err change influence:%f,dimpsi:%f",err_ago[0] - err_ago[err_pre - 1], dimpsi);
                            off_shift_std = 1;
                            go_flag = 1;
                            go_cross_flag = 0;      // 内侧滑判据标志词
                            go_cross = err - go_imp_mean - dimpsi;       // 农机与农具的初始相对位置
                            errn_std = err - shift_std;
                            LOGI("car shift on:go_over_ratio:%f,go_imp_mean:%f,go_over_max:%f,shift_std:%f,go_flag:%d",
                                 go_over_ratio, go_imp_mean, go_over_max, shift_std, go_flag);
                        }

                        // 农机航偏持续大于0.02rad(1.146°) 中途农机摆正但农具快速回正: 农机导航微超调法
                        if (car_shift == 0) {                           // 未启用农机移线法 初始标定shift_std为0
                            go_cnt = 0;
                            go_cross_flag = 0;            // 内侧滑判据标志词
                            err_std = err;
                            errImp_std = errImp;          // 后续入线时用于越界判断
                            shift_std = 0;                // 原始导航线即为目标导航线
                            shift_std_ago = 0;
                            off_shift_std = 1;
                            go_flag = 1;                  // 启用标志词，防止重复标定农机具位置
                            LOGI("car_shift off:go_over_ratio:%f,go_over_max:%f,shift_std:%f,go_flag:%d",
                                 go_over_ratio, go_over_max, shift_std, go_flag);
                        }
                    }

                    // 农机移线
                    if (go_flag == 1) {               // go_flag = 1  完成定线，启用农机具调整控制策略
                        go_cnt++;                     // 标记启动至入线时间
                        LOGI("go_cnt:%d", go_cnt);
                        // 外侧滑处理 农机已回线 + 农具横偏没有明显变化 + 农具横偏大于5cm
                        if (abs(err_n) < 0.015 && err_n != 0 && abs(dpsi) < 0.025 &&
                            abs(dimer[down_pos]) < 0.015 && go_cnt >= 20) {
                            go_flag = 0;        // 需要重新定线
                            car_shift = 0;
                            LOGI("Go Line slip_out");
                        }
                        double cross_dis = 0.02;
                        cross_dis = 0.15 * abs(errImp);
                        if(cross_dis > 0.08){
                            cross_dis = 0.08;
                        }
                        if(cross_dis < 0.008){
                            cross_dis = 0.008;
                        }
                        // 内侧滑处理 农机待移动距离小于农具距离1.8cm  car_shift = 1 设定初始cross_err
                        if (abs(err_n - errImp - dimpsi) > (cross_dis + 0.02) && err_n != 0) {        // >2cm后开始判断(区分刚起步)
                            go_cross_flag = 1;              // 修正阶段 待移动的农机横偏应当至少小于农具横偏1.8cm
                            LOGI("Go Line go_cross_flag:%d", go_cross_flag);
                        }
                        if (abs(err_n - errImp - dimpsi) < cross_dis && go_cross_flag == 1 && err_n != 0 &&
                            (shift_std == shift_std_ago)) {
                            err_std = err;
                            errImp_std = errImp;
                            // shift_std = shift_std_ago + 0.025 * errImp / abs(errImp);
                            if (errImp * dimpsi < 0) {
                                shift_std = err - go_ratio * (go_imp_mean + err_ago[0] - err_ago[err_pre - 1]+ dimpsi);        // 此处是否需要引入1s的农机横偏减少量影响
                                LOGI("go_line slip_in_1 shift std:%f", shift_std);
                            } else {
                                /*shift_std = err - go_ratio * go_imp_mean -
                                            (err_ago[0] - err_ago[err_pre - 1]) - 0.5 * dimpsi;*/
                                // 农具航偏与农机横偏变化的影响应当直接作用于农具横偏
                                shift_std = err - go_ratio * (go_imp_mean + err_ago[0] - err_ago[err_pre - 1] + 0.5 * dimpsi);
                                LOGI("go_line slip_in_2 shift std:%f", shift_std);
                            }
                            off_shift_std = 1;
                            go_cross_flag = 0;
                            errn_std = err - shift_std;
                            LOGI("Go Line slip_in_1 shift_std:%f,shift_std_ago:%f", shift_std,
                                 shift_std_ago);
                        }
                        // 农机与农具的相对位置应当不小于车辆摆正时农机与农具的初始相对位置 (考虑农具航向)
                        if (car_shift == 1 && abs(err - errImp - dimpsi) - abs(go_cross) < 0 && (go_cnt > 20 && (go_cnt - go_cnt_last) > 15)) {
                            err_std = err;
                            errImp_std = errImp;
                            // shift_std = shift_std_ago + 0.03 * errImp / abs(errImp);
                            if (errImp * dimpsi < 0) {
                                shift_std = err - go_ratio * (go_imp_mean + err_ago[0] - err_ago[err_pre - 1]+ dimpsi);        // 此处是否需要引入1s的农机横偏减少量影响
                                LOGI("go_line in_1 shift std:%f", shift_std);
                            } else {
                                /*shift_std = err - go_ratio * go_imp_mean -
                                            (err_ago[0] - err_ago[err_pre - 1]) - 0.5 * dimpsi;*/
                                // 农具航偏与农机横偏变化的影响应当直接作用于农具横偏
                                shift_std = err - go_ratio * (go_imp_mean + err_ago[0] - err_ago[err_pre - 1] + 0.5 * dimpsi);
                                LOGI("go_line in_2 shift std:%f", shift_std);
                            }
                            go_cross = err - go_imp_mean - dimpsi;
                            off_shift_std = 1;
                            errn_std = err - shift_std;
                            go_cnt_last = go_cnt;
                            LOGI("Go Line slip_in_2 shift_std:%f,shift_std_ago:%f,go_cnt:%d,go_cross:%f",
                                 shift_std, shift_std_ago,go_cnt,go_cross);
                        }
                    }

                    if (errn_std != errn_std_ago || shift_std != shift_std_ago || (shift_std == 0 && off_shift_std == 1)) {      // 新周期开启   周期交替
                        LOGI("Approaching errn_std:%f,errn_std_ago:%f", errn_std, errn_std_ago);
                        over_flag = 1;                 // 农具两阶段控制标志词重置
                    }

                    // 根据不同农具横偏量调整农机可超调量
                    if (off_shift_std == 1) {
                        go_overshoot = go_over_ratio * abs(go_imp_mean);      // 农机超调响应量 最大超调量6cm
                        if (abs(go_overshoot) > go_over_max) {
                            go_overshoot = go_over_max;
                        }
                        LOGI("go_overshoot:%f,go_over_max:%f,go_imp_mean:%f,errImp:%f",
                             go_overshoot, go_over_max, go_imp_mean, errImp);
                        off_shift_std = 0;
                    }
                }

                // 首次入线判定: 农机航向小于1.5°，农具横偏小于5cm  起步时农具车速延迟1~2s 1.4325°
                if (abs(dpsi) < 0.025 && abs(errImp) < 0.05 && car_shift == 1) {
                    five_flag = 1;
                    car_flag = 0;       shift_flag = 0;     across_flag = 0;
                    slip_out_flag = 0;  slip_in_flag = 0;
                    slip_out_trick = 0; slip_in_trick = 0;
                    shift_cout = 1;
                    car_online = 1;         //  接入移线与侧滑判据       切入在线阶段
                    // 各种标志位可以在此处进行复位  待定 slip_in_flag
                    err_std = err;
                    errImp_std = errImp;
                    double ratio_q = 0.8;
                    if(ImplementType == 1){
                        ratio_q = 0.75;
                    }
                    if(ImplementType == 2){
                        ratio_q = 0.8;
                    }
                    if (errImp * dimpsi < 0) {
                        // 考虑农具航向信息
                        shift_std = err - ratio_q * (errImp + 1.05 * dimpsi + (err_ago[0] - err_ago[err_pre - 1]));
                        LOGI("first_online_five_1 err:%f,(errImp + 1.05 * dimpsi):%f,derr:%f,shift_std:%f,",
                             err, (errImp + 1.05 * dimpsi), err_ago[0] - err_ago[err_pre - 1],
                             shift_std);
                    } else {
                        shift_std = err - (ratio_q + 0.05)* (errImp + err_ago[0] - err_ago[err_pre - 1]);
                        LOGI("first_online_five_2,err_std:%f,errImp:%f,deer:%f,shift_std:%f,shift_std_ago:%f",
                             err_std, errImp, err_ago[0] - err_ago[err_pre - 1], shift_std,
                             shift_std_ago);
                    }
                    LOGI("first_online_five");
                    // go_on_cmt = 1;          // 起效持续1.5s
                    errn_std = err - shift_std;
                }
            }
            err_n = err - shift_std;
            LOGI("Approach Line err_n:%f", err_n);
        }

        // 在线阶段: 微偏微调、小偏轻调   2.5cm 1.5cm 0.6cm  避免停车干扰 curv > 0.15m/s(0.54km/h)
        if ((Control_Type != 2) && car_online == 1) {
            if (abs(errImp) > 0.1) {      // 处理失衡后仍能重新入线
                go_flag = 0;
                car_shift = 0;
                car_online = 0;
            }
            /*if (go_on_cmt >= 1) {
                go_on_cmt++;
                if (go_on_cmt >= 15) {
                    go_on_cmt = 0;
                }
            }*/
            LOGI("online shift_std_ago:%f", shift_std_ago);
            if (curv > mini_vel_limit || (time_on == 1 && abs(curv) > mini_vel_limit)) {    // time_on == 1 倒车时也采用策略移线法
                // 微偏微调: 1.5cm  0.8cm  目标导航线附近处理措施 重点防超调 car_flag across_flag
                // 1.如何判定已经进入在线阶段 2.在线阶段如何移线微调 3.离线阶段如何切换
                if (abs(dpsi) < 0.013 && abs(dimpsi) < 0.013 && abs(errImp) < 0.013 &&
                    car_flag == 0) {
                    car_cmt = 0;            // 稳定稳态计数
                    car_flag = 1;           // car_flag = 1 过渡稳态   car_flag = 2 稳定稳态
                    // 策略目的: 提前回正防超调
                    if ((abs(err_n - errImp) < 0.01 ||
                         (err_n * errImp > 0 && abs(err_n) >= abs(errImp)))
                        && across_flag == 0 && err_n != 0 && (shift_std == shift_std_ago)) {
                        five_flag = 0;
                        shift_flag = 0;
                        slip_in_flag = 0;
                        slip_out_flag = 0;
                        err_std = err;
                        errImp_std = errImp;        // 农具横偏标志量，用于检测过线
                        if(abs(dimer[down_pos]) > 0.015 && (dimer[down_pos] * errImpago[0] < 0)){
                            shift_std = shift_std_ago + 0.0065 * errImp / abs(errImp);  // 提前回正防超调 削弱移线量
                            errn_std = err - shift_std;    // 定线后计算当前周期的初始errn_std，计算target_cout
                            LOGI("across_flag shift_std:%f,shift_std_ago:%f, errn_std:%f", shift_std,
                                 shift_std_ago, errn_std);
                        }

                        if (shift_cout == 0) {  // car_flag = 1 仅across_flag生效
                            shift_cout = 1;
                            LOGI("shift_cout 1");
                        }
                    }
                    LOGI("car_flag:%d,across_flag:%d", car_flag, across_flag);
                }
                if (car_flag == 1) {
                    car_cmt++;
                    if (car_cmt >= 15) {
                        car_cmt = 0;
                        car_flag = 2;       // 稳定稳态阶段
                        five_flag = 0;
                        shift_flag = 0;
                        across_flag = 0;
                        slip_in_flag = 0;
                        slip_out_flag = 0;
                        shift_cout = 0;
                        target_cout = 0;
                        LOGI("car_flag:%d,shift_cout:%d,flag_reset.", car_flag, shift_cout);
                    }
                    /*// 过渡阶段出现across_flag = 1(越界现象) 则沿用across_flag   =1/=0 进入car_flag = 1
                    if (across_flag == 1 && shift_cout <= 2 && car_cmt <= 10) {
                        car_cmt = 0;
                        LOGI("across_flag == 1 && shift_cout <= 2");
                    }*/
                }
                // 农机横偏超过1.5cm，重新启动移线及异常情况(侧滑)处理 将此前推倒重来
                if (abs(errImp) > 0.015 && car_flag != 0) {     // 避免反复进入shift_flag策略  反复进入shift_cout = 0
                    LOGI("tricks_start errImp:%f,car_flag:%d,car_cmt:%d", errImp, car_flag,car_cmt);
                    car_cmt = 0;        // 记录在线停留时间 结合across进行入线初期时的越界判定
                    car_flag = 0;
                    shift_cout = 0;
                    target_cout = 0;
                    five_flag = 0;
                    shift_flag = 0;
                    across_flag = 0;
                    slip_in_flag = 0;
                    slip_out_flag = 0;
                }

                // 理论移动距离 结合农机是否处于在线阶段(微偏微调) 农具横偏分段，预留调整空间防超调
                if (car_flag != 2) {
                    if (abs(errImp) < 0.02) {
                        shift_R = shift_R1; // 1.5cm ~ 2.5 cm
                    }
                    if (abs(errImp) >= 0.02 && abs(errImp) < 0.035) {
                        shift_R = shift_R2;
                    }
                    if (abs(errImp) >= 0.035) {
                        if(abs(curv) < 2.25){
                            shift_R = shift_R2 + 0.15;
                        }
                        if(abs(curv) < 2.75 && abs(curv) >= 2.25){
                            shift_R = shift_R2 + 0.1;
                        }
                        else{
                            shift_R = shift_R2 + 0.05;
                        }
                        if(shift_R >= 0.85){shift_R = 0.85;}
                    }
                }
                // 微偏微调  移动距离系数改为0.2 移动周期改为2s 移动力度改为超调0.5cm
                if (car_flag == 2) {
                    shift_R = 0.45;          // 0cm ~ 1.5cm    开放参数
                    if(abs(curv) > 2.75){
                        shift_R = 0.35;
                    }
                }
                LOGI("car_flag:%d,curv:%f,shift_R1:%f,shift_R2:%f,shift_R:%f", car_flag, curv, shift_R1, shift_R2, shift_R);

                // 如何处理在线阶段农机横偏引起的农具横偏问题  针对点:shift_flag!    待定

                // across_line:调整农具位置阶段 目标导航线附近处理措施  1.首次越界  2.越界1cm
                // 2s的横偏变化量，若减小超过1.5cm，则认为正处于快速收线阶段 越界惩罚将更加严厉
                if (errImp * errImp_std < 0 && across_flag == 0 && abs(errImp) >= 0.0036) {
                    int across_std = 0;
                    across_flag = 1;
                    five_flag = 0;
                    shift_flag = 0;
                    slip_in_flag = 0;
                    slip_out_flag = 0;
                    LOGI("across line.");

                    err_std = err;
                    errImp_std = errImp;
                    // 目的：归一到以当前errImp作重新定线 errImp必然较小
                    if ((err_n * errImp < 0 ||
                         (abs(err_n) < 0.005) && err_n != 0)) {   // 原err_n大于0.5cm
                        across_std = 1;
                        shift_std = err - 0.0065 * errImp / abs(errImp);
                        shift_cout = 1;
                        LOGI("shift_cout 2");
                        errn_std = err - shift_std;
                        LOGI("err_n * errImp < 0 || abs(err_n) < 0.005 shift_std:%f, errn_std:%f,errImp:%f,err_n:%f",
                             shift_std, errn_std, errImp, err_n);
                    }

                    // 判断农具横偏变化趋势 横偏呈现逐渐减小的趋势 增大重新定线力度
                    if (abs(dimer[down_pos]) > 0.015 && (dimer[down_pos] * errImpago[0] < 0) && (across_std = 1)
                        && abs(err_n) < 0.01 && err_n != 0) {
                        shift_std = shift_std - 0.005 * errImp / abs(errImp);
                        shift_cout = 1;
                        LOGI("shift_cout 3");
                        errn_std = err - shift_std;
                        LOGI("down_flag abs(dimer[down_pos]):%f errImpago[0]:%f shift_std:%f, errn_std:%f",
                             abs(dimer[down_pos]), errImpago[0], shift_std, errn_std);
                    }
                }
                // 农具当前横偏与标志横偏同号，且侧滑超过1cm，处于越界阶段，则加大移线距离
                if (errImp * errImp_std > 0 && abs(errImp) - abs(errImp_std) > 0.01 &&
                    across_flag == 1 && (abs(err_n) < 0.005) && err_n != 0
                    && car_flag != 2 && (shift_std == shift_std_ago)) {
                    err_std = err;
                    errImp_std = errImp;
                    shift_std = shift_std_ago - 0.005 * errImp / abs(errImp); // 预留调整空间防超调
                    shift_cout = 1;
                    LOGI("shift_cout 4");
                    errn_std = err - shift_std;
                    LOGI("slip out one shift_std:%f, errn_std:%f", shift_std, errn_std);
                }
                // 越界超过1.5cm，开启新周期  必须设置trick策略的跳出逻辑(进入--跳出)
                if (abs(errImp) > 0.015 && across_flag == 1) {
                    across_flag = 0;
                    LOGI("abs(errImp) > 0.015 && across_flag == 1");
                }

                // 多种tricks策略 移线周期(频率 刷新) 移线计数
                if (shift_cout >= target_cout && (target_cout >= 2)) {     // min target_cout 1
                    if (car_flag == 1 && car_cmt >= 1) { shift_cout++; }  // 过渡稳态car_flag 1进入稳态2处理
                    else { shift_cout = 0; }
                    target_cout = 0;
                    five_flag = 0;
                    shift_flag = 0;
                    across_flag = 0;
                    slip_in_flag = 0;
                    slip_out_flag = 0;
                    LOGI("shift_cout >= target_cout:shift_cout:%d,car_cmt:%d", shift_cout, car_cmt);
                }

                // 较大偏差下农机移动响应时间长，期间农具可能会发生不同程度的侧滑！ ---> 迭代移动周期解算及计数策略
                // 农机剩余移线距离小于1cm tricks策略标志位为1 移线计数开启
                if (((five_flag == 1) || (shift_flag == 1) || (slip_in_flag == 1) || (across_flag == 1) ||
                     (slip_out_flag == 1) || (car_flag != 0)) && (abs(err_n) < 0.01) && err_n != 0) {
                    if (target_cout != 0) {
                        shift_cout = shift_cout + 1;
                        LOGI("shift_cout = shift_cout + 1 :%d", shift_cout);
                    }
                }

                // across_flag = 1周期内可能出现abs(err_n) > abs(errImp)，避开对侧滑策略的影响。
                if (across_flag == 0) {
                    // 农具在线 微偏微调  稳定稳态阶段 shift_flag
                    // 农具横偏变化趋势：shift_flag、across_flag、slip_in_flag
                    double small_upper = 0.0085;
                    if (car_flag == 2) {
                        shift_value = 0.006;
                        ago_L = 5;              // 缩短用于判断的数据长度
                        if(abs(errImp) <= shift_value){
                            ago_L = 8;
                        }
                    }
                    if (car_flag == 0) {
                        shift_value = 0.01;
                        ago_L = 10;            // 数据长度可以设置为可调参数  判断待定
                    }
                    if(abs(curv) >= 2.5){
                        if(ImplementType == 1){
                            shift_value = shift_value + 0.002;
                            small_upper = small_upper + 0.002;
                        }
                        if(ImplementType == 2){
                            shift_value = shift_value + 0.0035;
                            small_upper = small_upper + 0.0035;
                        }
                    }
                    LOGI("shift_value:%f,ago_L:%d,car_flag:%d", shift_value, ago_L, car_flag);
                    shift_cnt = 0;  // 放在判断之外赋初值值为0，可以避免上周期遗留数值影响  ago_L:数据长度
                    if (shift_cout == 0 && abs(curv) > 0.05 && (ago_i <= -1) && (car_flag != 1)) {
                        LOGI("shift flag stage start.");
                        for (int jj = 0; jj <= ago_L; jj++) {
                            if (abs(errImpago[jj]) >= shift_value ||
                                (car_flag_ago == 2 && car_flag != 2) ||
                                ((abs(errImpago[jj]) <= shift_value) && car_flag == 2 && abs(err_n) >= 0.0045)) {
                                shift_cnt = shift_cnt + 1;
                                if (shift_cnt >= ago_L || (abs(errImp) >= 0.0085 && car_flag == 2) || (abs(errImp) >= 0.015)) {
                                    // 计算1s内(在线阶段 -- 0.5s)农具横偏平均值 并与当前值比较 取小
                                    imp_ten_mean = 0.0, imp_ten_sum = 0.0;
                                    for (int kk = 0; kk <= ago_L - 1; kk++) {
                                        imp_ten_sum += errImpago[kk];
                                    }
                                    imp_ten_mean = imp_ten_sum / (1.0 * ago_L);   // 计算算术平均值 浮点数转换
                                    LOGI("imp_ten_sum:%f, imp_ten_mean:%f.", imp_ten_sum,
                                         imp_ten_mean);

                                    imp_five_mean = 0, imp_five_sum = 0.0;
                                    for (int ii = 0; ii < 5; ii++) {
                                        imp_five_sum += errImpago[ii];
                                    }
                                    if (ago_L > 5) {
                                        imp_five_mean = imp_five_sum / 5.0;   // 计算算术平均值 浮点数转换
                                        LOGI(" imp_five_sum:%f, imp_five_mean:%f.", imp_five_sum,
                                             imp_five_mean);
                                        imp_mean = 0.3 * imp_ten_mean + 0.7 * imp_five_mean;
                                    } else {
                                        imp_mean = imp_ten_mean;
                                    }
                                    min_val = imp_mean;
                                    // 增加在线阶段的调整力度与节奏       (abs(errImp) >= 0.01 && car_flag == 2)
                                    if ((abs(errImp) < abs(min_val)) || (abs(errImp) >= 0.0085 && car_flag == 2)) {
                                        min_val = errImp;
                                    }
                                    int errImp_kk = 0;
                                    for(int i = 0; i <= 9; i++){
                                        if(abs(errImpago[i]) <= 0.006){
                                            errImp_kk ++;
                                        }
                                        if(errImp_kk >= 6 && abs(errImp) >= 0.006){
                                            min_val = errImp;
                                        }
                                    }
                                    shift_dis = shift_R * min_val;   // 解算农机待移动距离
                                    LOGI("shift_dis:%f, min_val:%f", shift_dis, min_val);

                                    if (err_n * errImp < 0 &&
                                        (shift_std == shift_std_ago)) {       // 判断前应当计算err_n
                                        // 移动周期解算有误时，尚未调节的err_n将叠加至当前移线量，农具横偏影响重复致使超调
                                        shift_std = shift_std_ago - shift_dis; // 将上一阶段的农机移动引入当前阶段    待定
                                        LOGI("shift_1 shift_std:%f", shift_std);
                                    } else {
                                        shift_std = err - shift_dis - shift_R * (err_ago[0] - err_ago[err_pre - 1]);
                                        LOGI("shift_2 shift_std:%f", shift_std);
                                    }
                                    if (shift_flag == 1 && car_flag_ago == 2 &&
                                        (shift_std == shift_std_ago)) {   // 微偏微调期间 削弱上周期的影响
                                        shift_std = shift_std_ago - shift_R * (err_ago[0] - err_ago[err_pre - 1]);
                                        LOGI("shift_3 shift_std:%f", shift_std);
                                    }
                                    if(abs(errImp) <= 0.006){
                                        shift_std = err - shift_dis- shift_R * (err_ago[0] - err_ago[7]);
                                        LOGI("mini errImp adjust.");
                                    }

                                    shift_flag = 1;
                                    shift_cout = 1;
                                    LOGI("shift_cout 5");
                                    five_flag = 0;
                                    slip_in_flag = 0;
                                    slip_out_flag = 0;

                                    err_std = err;
                                    errImp_std = errImp;    // 农具横偏标志量，用于检测过线

                                    // trick首次农机解算横偏 errn_std 参与解算移线周期
                                    errn_std = err - shift_std;     // 移线周期交替标志词
                                    LOGI("shift_flag:%d,shift_cout:%d", shift_flag, shift_cout);
                                }
                            }
                        }
                    }

                    // 微偏微调阶段,防侧滑策略不生效      // err_n如果不减小，出现增大现象，如何处理
                    if (car_flag != 2 && err_n != 0) {
                        // slip_out_flag 向外侧滑场景(农具横偏增大，可能出现大偏差(大缓弯)，交接行差)
                        // if (shift_cout < 0.65 * target_cout) { eso_cnt = 0; }    弃用计数判定
                        // 1.errImp_dis > 0.013     2.eso微补偿机制 农具的移动距离至少高于2s前的农机移动距离

                        if (Imp_out_cnt >= 1) {   // 开启新的移线周期后,农具横偏先增大后减小，农机和农具的相对位置先增大后减小
                            /*if (errImp_out_flag == 1 || Imp_out_cnt == 1) {   // errImp_dis > 0.013情形刷新errImp_out
                                errImp_out = errImp;        // 农具大侧滑标志量，用于防侧滑
                                errImp_out_flag = 0;
                            }*/
                            Imp_out_cnt++;  // 外侧滑新周期启动即开始计数      // 调节后反应时间与持续调节的矛盾   待定
                            errImp_dis = abs(errImp) - abs(errImp_out);   // 情形二判断期间，应当仍保留情形一的判断
                            double out_value = 0.013;
                            if(ImplementType == 1){
                                out_value = 0.012;
                            }
                            if(ImplementType == 2){
                                out_value = 0.0145;
                            }
                            if (errImp_dis > out_value) {
                                // 前2s内农机移动横偏将会产生的影响作用于当前的errImp，(减小)移动距离  映射系数调节
                                shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre-1]);
                                LOGI("slip out_1 shift_std:%f,%f,%f,%f", shift_std, err,
                                     (err_ago[0] - err_ago[err_pre - 1]), shift_R * errImp);
                                slip_out_trick = 1;
                                // errImp_out_flag = 1;        // 农具大侧滑标志量，时刻保留大偏差的判据，防大侧滑
                                LOGI("before 2s, occur slip outside.");
                            }
                            if(abs(car_state_ago) < 0.01 && abs(errImp) >= 0.026 && shift_R >= 0.3){
                                slip_out_cmt ++;
                                if(slip_out_cmt >= 9){
                                    slip_out_cmt = 0;
                                    // 处理前2s农机移动距离将产生的影响 并重新定线
                                    shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre - 1]);
                                    slip_out_trick = 2;
                                }
                            }

                            if(abs(car_state_ago - errImp) >= 0.02 && abs(errImp) >= 0.035 && shift_R >= 0.3){
                                slip_out_cnt ++;
                                if(slip_out_cnt >= 9){
                                    slip_out_cnt = 0;
                                    // 处理前2s农机移动距离将产生的影响 并重新定线
                                    shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre - 1]);
                                    slip_out_trick = 3;
                                }
                            }
                            // 连续一段时间出现slip_out_trick = 2/3现象，及时清零避免对之后判断产生干扰
                            if(abs(errImp) < 0.035){
                                slip_out_cnt = 0;
                                if(abs(errImp) < 0.026){
                                    slip_out_cmt = 0;
                                }
                            }
                            if(abs(car_state_ago - errImp) < 0.02){
                                slip_out_cnt = 0;
                            }
                            if(abs(car_state_ago) >= 0.01){
                                slip_out_cmt = 0;
                            }

                            if (Imp_out_cnt <= 2) {
                                max_pos = errImp;
                            }
                            if (abs(max_pos) < abs(errImp) && max_pos_flag == 0) {
                                max_pos = errImp;
                            }
                            if (abs(max_pos) - abs(errImp) > 0.005) {
                                max_pos_flag = 1;
                            }
                            if (max_pos_flag == 1) {
                                if(abs(errImp) - abs(max_pos) > 0.002){  // 增加一个与0.5s前判定 若农具横偏持增加，则重新定线
                                    // 处理前2s农机移动距离将产生的影响 并重新定线  横偏增大，大于单次调节的历史最值
                                    shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre - 1]);
                                    slip_out_trick = 4;
                                }
                                int out_cmt = 0;              // 横偏增大，大于历史值
                                for(int i = 1;i <= 5;i++){
                                    if(abs(errImp) - abs(errImpago[i]) > 0.001){
                                        out_cmt++;
                                    }
                                    if(out_cmt >= 3 && abs(errImp) - abs(errImpago[5]) > 0.0035){
                                        // 处理前2s农机移动距离将产生的影响 并重新定线
                                        shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre - 1]);
                                        slip_out_trick = 5;
                                    }
                                }
                            }
                            LOGI("slip_out_cnt:%d,slip_out_cmt:%d,slip_out_trick:%d",slip_out_cnt,slip_out_cmt,slip_out_trick);
                            LOGI("Imp_out_cnt:%d,errImp_dis:%f,err_pre:%d", Imp_out_cnt, errImp_dis,err_pre);
                            // 农具横偏呈现减小趋势后，可增加 errImp与 errImp_cal的比较  // 此处设置侧滑策略标志词  方便统一  待定
                            if (slip_out_trick == 1 || slip_out_trick == 2 || slip_out_trick == 3 || slip_out_trick == 4 || slip_out_trick ==5) {
                                err_std = err;
                                errImp_std = errImp;                // 农具横偏标志量，用于检测过线
                                shift_cout = 1;
                                LOGI("shift_cout 6");
                                shift_flag = 0;
                                five_flag = 0;
                                slip_in_flag = 0;
                                slip_out_flag = 1;
                                slip_out_trick = 0;
                                slip_out_trick = 0;
                                errn_std = err - shift_std;   // trick首次农机解算横偏 errn_std 参与解算移线周期
                                LOGI("big slip log errn_std:%f", errn_std);
                            }
                        }

                        // 向内侧滑(农具横偏快速减小，可能出现超调(小碎弯)，直线度差)移线阶段享有最高优先级 弃用计数判定
                        // 1.农机与农具相对位置相较于首次定线判据，农机与农具之间的距离不应该小于初始相对距离 判定应当激进  此处逻辑有误 调节期间相对位置就应当减小
                        // 2.除非err_n < 1cm且保持1.5s以上，否则农机解算值与农具的位置偏差应当至少在1cm以上 此处合理 预留农具响应农机变化的移动空间
                        // 3.引入农具横偏变化趋势，调整防止超调的力度。
                        // 关注分析农机与农具相对位置相较于初始值的增大量与农具横偏的变化关系

                        // 微偏微调阶段，errn_one很小  增加初始相对距离判断，调整-0.01数值
                        if (Imp_in_flag == 1) {          // 倘若持续侧滑如何处理？  待定
                            if (abs(errImp_std) < 0.015) {
                                cross_in_num = -0.65 * abs(errImp_std);
                            } else {
                                cross_in_num = -0.007;          // 设置为可调参数
                            }
                            // 比较变化量：农机入线(<1cm)1.5s前,农机解算值与农具的位置偏差应当至少在1cm以上  // err_n与errImp异号如何处理  排除轻微变化干扰
                            slip_in_std = 0;
                            // 比较实时量
                            if ((((abs(err_n) - abs(errImp) > cross_in_num && err_n * errImp > 0) &&
                                  (abs(errImp) >= 0.02) && (shift_std == shift_std_ago) && abs(err_n) >= 0.0065) ||
                                 (abs(err_n) - abs(errImp) > 0 && err_n * errImp > 0)) &&
                                (car_cmt <= 10) && abs(errImp) > 0.0036) {  // 较大偏差回线防侧滑
                                // 处理前2s农机移动距离将产生的影响 并重新定线  充分考虑外侧滑策略的影响
                                if (slip_out_flag == 0 || (slip_out_flag == 1 && (abs(max_pos) - abs(errImp) > 0.007) &&
                                     Imp_out_cnt >= 10) || (abs(err_n) - abs(errImp) > 0.025)) {
                                    slip_in_std = 1;
                                    slip_in_trick = 1;
                                    shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre - 1]);
                                    // shift_std 如何更新需要谨慎思考
                                    LOGI("err_n:%f occur slip inside.shift_std:%f,cross_in_num:%f",
                                         err_n, shift_std, cross_in_num);
                                    if(abs(err_n) - abs(errImp) > 0 && err_n * errImp > 0 && (car_cmt <= 10)) {
                                        slip_in_std = 1;
                                        slip_in_trick = 2;
                                        // 处理前2s农机移动距离将产生的影响 并重新定线
                                        shift_std = err - shift_R * (errImp + err_ago[0] - err_ago[err_pre - 1]);
                                        LOGI("slip in even.err_n:%f,errImp:%f,err_ago[0]:%f,err_ago[9]:%f",
                                             err_n, errImp, err_ago[0], err_ago[err_pre - 1]);
                                    }
                                }
                            }
                            // 判断农具横偏变化趋势 横偏呈现逐渐减小的趋势  增加惩处力度
                            if (abs(dimer[down_pos]) > 0.015 && (dimer[down_pos] * errImpago[0] < 0) &&
                                slip_in_std == 1) {
                                slip_in_std = 0;
                                shift_std = shift_std + 0.005 * errImp / abs(errImp);   // 增加移线量 不必再调整移线时间
                                LOGI("down_flag_2 shift_std:%f, dimer[down_pos]:%f,errImpago[0]:%f",
                                     shift_std, dimer[down_pos], errImpago[0]);
                            }
                            if (slip_in_trick == 1 || slip_in_trick == 2) {
                                slip_in_trick = 0;
                                slip_in_flag = 1;
                                shift_flag = 0;
                                five_flag = 0;
                                slip_out_flag = 0;
                                shift_cout = 1;
                                LOGI("shift_cout 7");
                                err_std = err;
                                errImp_std = errImp;
                                errn_std = err - shift_std;   // trick首次农机解算横偏 errn_std 参与解算移线周期
                                LOGI("slip in errn_std:%f", errn_std);
                            }
                        }
                    }
                }
                // 一、调节周期 = 农机移动周期(难以准确估计) + 农具时滞周期 ×; 二、调节周期 abs(err_n) < 0.01 ✔
                // 周期解算策略：err_n < 1cm 开始计数  1cm-1m/s  0.8cm 1m/s --3s
                if (errn_std != errn_std_ago) {      // 新周期开启   周期交替
                    LOGI("next cycle errn_std:%f,errn_std_ago:%f", errn_std, errn_std_ago);
                    over_flag = 1;                 // 农具两阶段控制标志词重置
                    Imp_out_cnt = 1;               // 新周期外侧滑计数重置
                    Imp_in_flag = 1;               // 新周期内侧滑判据开启
                    errImp_out = errImp;        // 农具大侧滑标志量，用于防侧滑
                    max_pos = 0;        max_pos_flag = 0;
                    slip_out_cmt = 0;   slip_out_cnt = 0;
                    if(car_flag != 2){
                        if(car_cmt >= 5){
                            car_cmt = 5;
                        }
                        if(car_cmt < 5 || car_flag == 0){
                            car_cmt = 0;
                        }
                    }
                    // 调节周期计算 农具响应滞后时间
                    if (abs(errn_std) > 0.01) {
                        if (Control_Type <= 2) {
                            delay = GainLearnIm / 1000 - Control_Type * 100 - GainLearnIm/1000 % 5;
                            shift_f = abs(delay);  // 默认值25
                            LOGI("shift_f_0:%f",shift_f);
                        } else {
                            shift_f = 25;                           // 车速自适应  待定
                            LOGI("shift_f_1:%f",shift_f);
                        }
                    }
                    if (abs(errn_std) < 0.01) {
                        if (time_on <= 3) {
                            delay = PTimeIm / 10 - time_on * 100;
                            shift_f = abs(delay * 100 * errn_std);
                            LOGI("shift_f_2:%f",shift_f);
                        } else {
                            shift_f = abs(2000 * errn_std);
                            LOGI("shift_f_3:%f",shift_f);
                        }
                    }

                    double mode_ratio = 1;
                    if(abs(curv) < 1.25){
                        if(ImplementType == 1){
                            mode_ratio = 1.15;
                        }
                        if(ImplementType == 2){
                            mode_ratio = 1.25;
                        }
                    }
                    shift_f = shift_f * mode_ratio;
                    if (curv > 0.5) {
                        target_cout = static_cast<int>(shift_f / curv);  // 目标计数与车速的数量关系
                    } else {
                        target_cout = static_cast<int>(shift_f);  // 目标计数与车速的数量关系
                    }
                    if (target_cout < 15 && abs(errn_std) > 0.01) { target_cout = 15; }
                    if (target_cout < 10 && abs(errn_std) < 0.01) { target_cout = 10; }
                    if(ImplementType == 2 && abs(curv) > 2.25){
                        target_cout = target_cout + 5;
                    }
                    LOGI("target_cout:%d,shift_f:%f,errn_std:%f", target_cout, shift_f,errn_std);
                }
            }
            // 解算的农机移线横偏
            err_n = err - shift_std;
            LOGI("On Line err_n:%f,shift_std:%f", err_n, shift_std);
        }
        if (curv < 0) {
            err_n = err - shift_std;
        }

        if(abs(errImp) < 0.0125){
            overshoot = 0.006 - abs(curv) * 0.00125;
            if(overshoot < 0.0025){
                overshoot = 0.0025;
            }
        }
        // 移线力度 农机超调加快收敛 Smith预估特点 go_overshoot overshoot
        if (over_flag == 1) {     // 每一次刷新周期时，都应将 over_flag = 0
            if (car_online == 0) {
                car_state(1) = err_n + go_overshoot * errImp / abs(errImp);// err_n / abs(err_n)处理有误
                LOGI("car_state(1)_0:%f, err_n:%f, go_overshoot:%f", car_state(1), err_n,go_overshoot);
            }
            if (car_online == 1) {
                car_state(1) = err_n + overshoot * errImp / abs(errImp);
                LOGI("car_state(1)_1:%f, err_n:%f, overshoot:%f", car_state(1), err_n, overshoot);
                // 缓解常规大/小肚弯变成农具导航大/小S弯,加快农机调整速度
                if(across_flag ==1 || slip_out_flag == 1){
                    if(across_flag == 1 && slip_out_flag == 0){
                        pre_shoot = pre_shoot * 0.75;
                    }
                    car_state(1) = car_state(1) + pre_shoot * errImp / abs(errImp);
                }
            }
        } else {
            car_state(1) = err_n;
            LOGI("car_state(1) = err_n:%f", err_n);
        }
        // 1.检验农机导航算法  2.倒车  纯控农机导航
        if ((Control_Type == 2) || ((time_on == 2) && (curv < 0))) {
            car_state(1) = err;
            LOGI("car_state(1) = err:%f", car_state(1));
        }

        if(imp_mode == 22){                 // 平板显示的横偏数字
            if(on_line == 3){
                err_eso = car_state(1) - err;
            }
            else{
                err_eso = err_n - err;
            }
        }

        // 大偏差 高速入线策略
        if(car_online == 0 && imp_mode == 22 && gain_r <= 3){
            if(abs(errImp) <= 0.6){errImp_max = 0.6;}
            if(abs(errImp) > 0.6){errImp_max = 0.6 + 0.15 * (abs(errImp) - 0.6);}
            if(ImplementType == 1){
                if(abs(errImp) <= 0.5){errImp_max = 0.5;}
                if(abs(errImp) > 0.5){ errImp_max = 0.5 + 0.1 * (abs(errImp) - 0.5);}}
            if(cur_state.v > 1.75){
                errImp_max = errImp_max - (cur_state.v - 0.5) * (0.12 + (PTimeIm/10) % 5 * 0.015);
            }

            if(abs(car_state(1)) > errImp_max){
                errImp_upper = errImp_max;
                car_state(1) = errImp_max * car_state(1)/abs(car_state(1));
            }
        }

        if(imp_mode != 22 ||(imp_mode == 22 && on_line == 5)){          // 平板显示的横偏数字
            err_eso = car_state(1) - err;       //  13模式叠加偏差的符号  待定  返回的数值与目标导航线数值多数一致
        }
        car_state_ago = car_state(1);

        // Car_delta 农机移线求解控制量
        LOGI("Controller Type:Car_plant");
        LQR_car Car_plant(700, 1e-6, 0.1);
        q1 = GainxHeadingIm_mode - heading * 100;
        q2 = GainxTrackIm_mode;
        r = GainRIm_mode;
        if(car_online == 0 && abs(curv) > 1.75){
            q1 = q1 + static_cast<int> (8 * (abs(curv) - 0.5));     // 减缓入线速度防超调
            if(heading == 1){
                q2 = q2 - static_cast<int> (10 * (abs(curv) -1));        // 减缓入线速度防超调
                r = r + static_cast<int> (5 * (abs(curv) -1));
            }
            if(heading == 0){
                q2 = q2 + static_cast<int> (10 * (abs(curv) -1));        // 减缓入线速度防超调
                r = r - static_cast<int> (5 * (abs(curv) -1));
            }
            LOGI("reduce go line speed.");
        }

        car_state(0) = dpsi;
        LOGI("diff_state={%lf,%lf},err_n:%f", car_state(0), car_state(1), err_n);
        Car_plant.update_car_state(curx, cury, cur_psi, curv);
        Car_plant.update_imp_state(impx, impy, imp_psi, impv);
        Car_plant.Update_A_B_matrix(l1);
        Car_plant.Update_Q_R_matrix(q1, q2, r, 0.01, heading);
        LOGI("control system parameter");
        delta_LQR = Car_plant.CALC(&car_state, &K_car); //实际控制量  LQR -Kx

        if (curv < 0 && gain_r == 3 && heading <= 1) {
            delta_LQR = (GainRIm_mode - 350) / 100.0 * dpsi + delta_LQR;
        }

        car_K1 = Car_plant.CarData.car_K0;      car_K2 = Car_plant.CarData.car_K1;
        car_U = Car_plant.CarData.car_U0;       q1 = Car_plant.CarData.car_q1;
        q2 = Car_plant.CarData.car_q2;          r = Car_plant.CarData.car_r;
        // 小偏差期间尤其注重航向稳定，避免车身旋转导致的超调现象
        if(car_online == 1 && abs(err_n) <= 0.008 && abs(errImp) <= 0.0135 && err_n != 0 && over_flag == 0
            && curv >= mini_vel_limit && (abs(dpsi) < 0.0065 || (abs(dpsi) >= 0.0065 && dpsi * car_state(1) < 0))){
            dpsi_ratio = GainxHeadingIm % 5 * 0.125;
            if(GainxHeadingIm % 5 == 0){
                dpsi_ratio = 0.25;
            }
            if(GainxHeadingIm % 5 == 2){
                dpsi_ratio = 0;
            }
            if(dpsi_ratio / car_K1 > 0.25){
                dpsi_ratio = car_K1 * 0.25;
            }
            delta_LQR = delta_LQR - dpsi_ratio * dpsi;
            LOGI("GainxHeadingIm_mode 5 %d, dpsi_ratio:%f,car_K1/4:%f",GainxHeadingIm_mode % 5, dpsi_ratio, car_K1*0.25);
        }
        // 加速航偏回线收敛速度
        if(car_online == 1 && abs(car_state(1)) <= 0.02 && abs(dpsi) > 0.02 && dpsi * car_state(1) < 0){
            dpsi_back = OnlineAggresivenessIm_mode % 5 * 0.125;
            if(OnlineAggresivenessIm_mode % 5 == 0){
                dpsi_back = 0.25;
            }
            if(OnlineAggresivenessIm_mode % 5 == 2){
                dpsi_back = 0;
            }
            if(dpsi_back / car_K1 > 0.25){
                dpsi_back = car_K1 * 0.25;
            }
            delta_LQR = delta_LQR - dpsi_back * dpsi;
            LOGI("OnlineAggresivenessIm_mode 5 %d, dpsi_back:%f,car_K1/4:%f",OnlineAggresivenessIm_mode % 5, dpsi_ratio, car_K1*0.25);
        }

        // Imp_delta 基于农机具模型计算的前轮转角控制量  仅仅作用于入线
        LOGI("Controller Type:imp_delta");
        LQR_imp_psi lqr_controller(700, 1e-6, 0.1);
        // 权重系数 自适应车速变化
        qq0 = 25, qq2 = 120, rr0 = 80;
        double qq1 = 0,rr2 = 0;
        qq0 = GainSteeringIm_mode - 100 *steer;    //data1.GainxHeading; 10            // 农机航向误差权重
        qq2 = GainxTrackIm_mode;      // data1.GainxTrack;  160           // 农具横向误差权重
        rr0 = GainRIm_mode;           // data1.GainR;       80            // 控制量权重
        if(car_online == 0 && abs(curv) > 1.75){
            qq0 = qq0 + static_cast<int> (8 * (abs(curv) - 0.5));     // 减缓入线速度防超调
            if(steer == 1){
                qq2 = qq2 - static_cast<int> (10 * (abs(curv) -1));        // 减缓入线速度防超调
                rr0 = rr0 + static_cast<int> (5 * (abs(curv) -1));
            }
            if(steer == 0){
                qq2 = qq2 + static_cast<int> (10 * (abs(curv) -1));        // 减缓入线速度防超调
                rr0 = rr0 - static_cast<int> (5 * (abs(curv) -1));
            }
            LOGI("reduce go line speed.");
        }

        Eigen::Vector3d imp_diff_state;
        imp_diff_state(0) = dpsi;
        imp_diff_state(1) = dimpsi;
        imp_diff_state(2) = errImp;
        LOGI("diff_state={%lf,%lf,%lf}", imp_diff_state(0), imp_diff_state(1), imp_diff_state(2));
        lqr_controller.update_car_state(curx, cury, cur_psi, curv);
        lqr_controller.update_imp_state(impx, impy, imp_psi, impv);
        lqr_controller.Update_A_B_matrix(l1, l2, l3, &imp_diff_state);
        lqr_controller.Update_Q_R_matrix(qq0, qq1, qq2, rr0, 0.1,steer);
        delta_imp = lqr_controller.CALC(&imp_diff_state, &K_imp,errImp_max); //实际控制量  LQR -Kx

        imp_K0 = lqr_controller.impData.imp_K0;    imp_K1 = lqr_controller.impData.imp_K1;
        imp_K2 = lqr_controller.impData.imp_K2;    imp_U0 = lqr_controller.impData.imp_U0;
        qq0 = lqr_controller.impData.q00;          qq2 = lqr_controller.impData.q22;
        rr0 = lqr_controller.impData.r00;          imp_max = lqr_controller.impData.imp_max;
        LOGI("l1:%f,l2:%f,l3:%f,qq0:%f,qq2:%f,rr0:%f,steer:%d,imp_K0:%f,imp_K1:%f,imp_K2:%f,delta_imp:%f",
             l1,l2,l3,qq0,qq2,rr0,steer,imp_K0,imp_K1,imp_K2,delta_imp);

        // 农机航向回正阶段
        if (((*RQ_ago) * dpsi < 0 && errImp * dpsi < 0) ||
            (abs(errImp) < 0.006 && car_online == 1) ||
            (err_n * errImp < 0 && abs(err_n) > 0.01)) {
            over_flag = 0;
        }

        if(imp_mode == 22){
            // 前进及倒车入线是否选择农机具模型控制方法。        用农机具模型法
            if(car_online == 0){
                *delta = delta_imp;
                if(curv < 0){
                    if(time_on == 3){
                        *delta = delta_imp;      // 倒车用农机具模型法
                    }
                    else{
                        *delta = delta_LQR;      //倒车用农机导航 + 农机移线
                    }
                }
                if(curv > 0){
                    if(car_shift == 0){
                        if(GainSteeringIm_mode % 5 != 1){
                            *delta = delta_imp;
                        }
                        if(GainSteeringIm_mode % 5 == 1){
                            *delta = delta_LQR;
                        }
                    }
                    if(car_shift == 1){
                        if(GainSteeringIm_mode % 5 == 2){
                            *delta = delta_imp;
                        }
                        else{
                            *delta = delta_LQR;
                        }
                    }
                }
            }
            if(car_online == 1){
                *delta = delta_LQR;
            }

            // 前轮转角最大限幅
            if ((*delta) > MaxSteeringAngle * 1.0 / 57.3)
                (*delta) = MaxSteeringAngle * 1.0 / 57.3;
            if ((*delta) < -MaxSteeringAngle * 1.0 / 57.3)
                (*delta) = -MaxSteeringAngle * 1.0 / 57.3;
        }
    }

    struct tm bj_time{};
    long double milliseconds;
    LQR_car::gps_to_beijing_time(Week, Second, &bj_time, &milliseconds);

    /*LOGI("GPS_time:week:%d, second:%f, Gps_UTCTime:%d\n", Week, Second / 100.0, Gps_UTCTime);
    LOGI("Beijing Time: %04d-%02d-%02d %02d:%02d:%02d.%03d\n",
         bj_time.tm_year + 1900,
         bj_time.tm_mon + 1,
         bj_time.tm_mday,
         bj_time.tm_hour,
         bj_time.tm_min,
         bj_time.tm_sec,
         (int) (milliseconds * 1000));*/

    /*LOGI("Date:%04d-%02d-%02d.%02d-%02d-%02d.%03d,Week:%d,Second:%f,"
         "DriverStatus:%d,ImplementType:%d,imp_mode:%d,Control_Type:%d,impv:%f,car_shift:%d,car_online:%d,err_eso:%f,delay:%d,err_pre:%d,shift_R:%f,shift_R1:%f,shift_R2:%f,shift_cout:%d,target_cout:%d,"
         "go_overshoot:%f,pre_shoot:%f,overshoot:%f,dpsi_ratio:%f,dpsi_back:%f,car_U:%f,delta_LQR:%f,delta:%f,delta_imp:%f,shift_std_ago:%f,shift_std:%f,errn_std:%f,errImp_std:%f,err:%f,err_n:%f,delta:%f,"
         "curv:%f,car_state(1):%f,errImp:%f,over_flag:%d,shift_f:%f,dpsi:%f,dimpsi:%f,rol:%f,imp_rol:%f,err_std:%f,five_flag:%d,shift_flag:%d,slip_out_flag:%d,slip_out_trick:%d,slip_in_flag:%d,slip_in_trick:%d"
         "across_flag:%d,car_flag:%d,car_cmt:%d,ago_L:%d,shift_cnt:%d,shift_value:%f,shift_dis:%f,min_val:%f,shift_f:%f,Pitch:%f,PitchIm:%f,errImp_upper:%f,errImp_max:%f,errImp_out:%f,q1:%f,q2:%f,r:%fcar_K1:%f,car_K2:%f,\n",
         bj_time.tm_year + 1900, bj_time.tm_mon + 1, bj_time.tm_mday, bj_time.tm_hour,bj_time.tm_min, bj_time.tm_sec, (int) (milliseconds * 1000), Week, Second / 100.0,
         DriverStatus, ImplementType, imp_mode, Control_Type, impv * 3.6, car_shift, car_online, err_eso*100, delay, err_pre,shift_R,shift_R1, shift_R2,shift_cout,target_cout,
         go_overshoot*100,pre_shoot*100,overshoot*100,dpsi_ratio,dpsi_back,car_U*57.3,delta_LQR*57.3,(*delta)*57.3,delta_imp*57.3,shift_std_ago*100,shift_std*100,errn_std*100,errImp_std*100,err*100,err_n*100,(*delta)*57.3,
         curv*3.6,car_state(1)*100,errImp*100,over_flag,shift_f,dpsi*57.3,dimpsi*57.3,rol*57.3,imp_rol*57.3,err_std*100,five_flag,shift_flag,slip_out_flag,slip_out_trick,slip_in_flag,slip_in_trick,
         across_flag,car_flag,car_cmt,ago_L,shift_cnt,shift_value*100, shift_dis*100,min_val*100,shift_f,Pitch*57.3/100,PitchIm*57.3/100,errImp_max*100,errImp_upper*100,errImp_out*100,q1,q2,r,car_K1,car_K2);
    LOGI("Date:dimer[0]:%f,dimer[1]:%f,dimer[%d]:%f,errImp_dis:%f,slip_out_cnt:%d,slip_out_cmt:%d,Imp_out_cnt:%d,cross_in_num:%f,back_stay_cnt:%d,back_cnt:%d,MaxSteeringAngle:%d,"
         "off_shift_std:%d,go_over_ratio:%f,go_over_max:%f,go_cross:%f,go_cross_flag:%d,go_ratio:%f,go_cnt:%d,go_flag:%d,go_stay_cnt:%d,go_imp_mean:%f,l1:%f,l2:%f,l3:%f,Aa:%f,Bb:%f,Cc:%f,curx:%f,cury:%f,impx:%f,"
         "impy:%f,T_psi:%f,cur_psi:%f,imp_psi:%f,mini_vel_limit:%f,Omiga:%f,IntegralSwitchIm:%d,GainSteeringIm_mode:%d,GainxTrackIm_mode:%d,GainxHeadingIm_mode:%d,GainRIm_mode:%d,GainLearnIm:%d,"
         "OnlineAggresivenessIm_mode:%d,ApproachAggresivenessIm_mode:%d,PTimeOffIm:%d,PTimeIm:%d\n",
         dimer[0]*100,dimer[1]*100,down_pos,dimer[down_pos]*100,errImp_dis*100,slip_out_cnt,slip_out_cmt, Imp_out_cnt,cross_in_num*100,back_stay_cnt,back_cnt,MaxSteeringAngle,
         off_shift_std,go_over_ratio,go_over_max,go_cross,go_cross_flag,go_ratio,go_cnt,go_flag,go_stay_cnt,go_imp_mean*100,l1,l2,l3,Aa,Bb,Cc,curx,cury,impx,
         impy,T_psi,cur_psi,imp_psi,mini_vel_limit*3.6,Omiga,IntegralSwitchIm,GainSteeringIm_mode,GainxTrackIm_mode, GainxHeadingIm_mode,GainRIm_mode, GainLearnIm/1000,
         OnlineAggresivenessIm_mode,ApproachAggresivenessIm_mode,PTimeOffIm/10,PTimeIm/10);*/
    // 关于前轮转角求解过程的打印信息，依据测试效果再考虑是否添加    待定
    if (fpControl != nullptr && MotorMoment == 99 && LogSwitch != 1) {
        if(go_line != 3){
            fprintf(fpControl,"Date:%04d-%02d-%02d.%02d-%02d-%02d.%03d,Week:%d,Second:%f,"
                              "%d, ,%d, ,%d, ,%d, ,%f, ,%d, ,%d, ,%f, ,%d, ,%d, ,%f, ,%f, ,%f, ,%d, ,%d, ,"
                              "%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,"
                              "%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%d, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%d, ,%d, ,%d, ,%d, ,%d, ,%d, ,"
                              "%d, ,%d, ,%d, ,%d, ,%d, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,"
                              "%f, ,%f, ,%f, ,%f, ,%d, ,%f, ,%f, ,%d, ,%d, ,%d, ,%f, ,%d, ,%d, ,%d, ,"
                              "%d, ,%f, ,%f, ,%f, ,%d, ,%f, ,%d, ,%d, ,%d, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,"
                              "%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%d, ,%d, ,%d, ,%d, ,%d, ,%d, ,"
                              "%d, ,%d, ,%d, ,%d\n",
                    bj_time.tm_year + 1900, bj_time.tm_mon + 1, bj_time.tm_mday, bj_time.tm_hour,bj_time.tm_min, bj_time.tm_sec, (int) (milliseconds * 1000), Week, Second / 100.0,
                    DriverStatus, ImplementType, imp_mode, Control_Type, impv * 3.6, car_shift, car_online, err_eso*100, delay, err_pre,shift_R,shift_R1, shift_R2,shift_cout,target_cout,
                    go_overshoot*100,pre_shoot*100,overshoot*100,dpsi_ratio,dpsi_back,car_U*57.3,delta_LQR*57.3,(*delta)*57.3,delta_imp*57.3,shift_std_ago*100,shift_std*100,errn_std*100,errImp_std*100,err*100,err_n*100,(*delta)*57.3,
                    errImp_max*100,errImp_upper*100,imp_max*100,curv*3.6,car_state(1)*100,errImp*100,over_flag,shift_f,dpsi*57.3,dimpsi*57.3,rol*57.3,imp_rol*57.3,err_std*100,five_flag,shift_flag,slip_out_flag,slip_out_trick,slip_in_flag,slip_in_trick,
                    across_flag,car_flag,car_cmt,ago_L,shift_cnt,shift_value*100, shift_dis*100,min_val*100,shift_f,Pitch*57.3/100,PitchIm*57.3/100,errImp_max*100,q1,q2,r,
                    car_K1,car_K2,dimer[0]*100,dimer[1]*100,down_pos,dimer[down_pos]*100,errImp_dis*100,slip_out_cnt,slip_out_cmt, Imp_out_cnt,cross_in_num*100,back_stay_cnt,back_cnt,MaxSteeringAngle,
                    off_shift_std,go_over_ratio,go_over_max,go_cross,go_cross_flag,go_ratio,go_cnt,go_flag,go_stay_cnt,go_imp_mean*100,l1,l2,l3,Aa,Bb,Cc,curx,cury,impx,
                    impy,T_psi,cur_psi,imp_psi,mini_vel_limit*3.6,Omiga,IntegralSwitchIm,GainSteeringIm_mode,GainxTrackIm_mode, GainxHeadingIm_mode,GainRIm_mode, GainLearnIm/1000,
                    OnlineAggresivenessIm_mode,ApproachAggresivenessIm_mode,PTimeOffIm/10,PTimeIm/10);
        }
        if(go_line == 3){
            fprintf(fpControl,"Date:%04d-%02d-%02d.%02d-%02d-%02d.%03d,Week:%d,Second:%f,"
                              "DriverStatus:%d,ImplementType:%d,imp_mode:%d,Control_Type:%d,impv:%f,car_shift:%d,car_online:%d,err_eso:%f,delay:%d,err_pre:%d,shift_R:%f,shift_R1:%f,shift_R2:%f,shift_cout:%d,target_cout:%d,"
                              "go_overshoot:%f,pre_shoot:%f,overshoot:%f,dpsi_ratio:%f,dpsi_back:%f,car_U:%f,delta_LQR:%f,delta:%f,delta_imp:%f,shift_std_ago:%f,shift_std:%f,errn_std:%f,errImp_std:%f,err:%f,err_n:%f,delta:%f,"
                              "errImp_max:%f,errImp_upper:%f,imp_max:%f,curv:%f,car_state(1):%f,errImp:%f,over_flag:%d,shift_f:%f,dpsi:%f,dimpsi:%f,rol:%f,imp_rol:%f,err_std:%f,five_flag:%d,shift_flag:%d,slip_out_flag:%d,"
                              "slip_out_trick:%d,slip_in_flag:%d,slip_in_trick:%d,across_flag:%d,car_flag:%d,car_cmt:%d,ago_L:%d,shift_cnt:%d,shift_value:%f,shift_dis:%f,min_val:%f,shift_f:%f,Pitch:%f,PitchIm:%f,errImp_out:%f,q1:%f,q2:%f,r:%f,"
                              "car_K1:%f,car_K2:%f,dimer[0]:%f,dimer[1]:%f,dimer[%d]:%f,errImp_dis:%f,slip_out_cnt:%d,slip_out_cmt:%d,Imp_out_cnt:%d,cross_in_num:%f,back_stay_cnt:%d,back_cnt:%d,MaxSteeringAngle:%d,"
                              "off_shift_std:%d,go_over_ratio:%f,go_over_max:%f,go_cross:%f,go_cross_flag:%d,go_ratio:%f,go_cnt:%d,go_flag:%d,go_stay_cnt:%d,go_imp_mean:%f,l1:%f,l2:%f,l3:%f,Aa:%f,Bb:%f,Cc:%f,curx:%f,cury:%f,impx:%f,"
                              "impy:%f,T_psi:%f,cur_psi:%f,imp_psi:%f,mini_vel_limit:%f,Omiga:%f,IntegralSwitchIm:%d,GainSteeringIm_mode:%d,GainxTrackIm_mode:%d,GainxHeadingIm_mode:%d,GainRIm_mode:%d,GainLearnIm:%d,"
                              "OnlineAggresivenessIm_mode:%d,ApproachAggresivenessIm_mode:%d,PTimeOffIm:%d,PTimeIm:%d\n",
                    bj_time.tm_year + 1900, bj_time.tm_mon + 1, bj_time.tm_mday, bj_time.tm_hour,bj_time.tm_min, bj_time.tm_sec, (int) (milliseconds * 1000), Week, Second / 100.0,
                    DriverStatus, ImplementType, imp_mode, Control_Type, impv * 3.6, car_shift, car_online, err_eso*100, delay, err_pre,shift_R,shift_R1, shift_R2,shift_cout,target_cout,
                    go_overshoot*100,pre_shoot*100,overshoot*100,dpsi_ratio,dpsi_back,car_U*57.3,delta_LQR*57.3,(*delta)*57.3,delta_imp*57.3,shift_std_ago*100,shift_std*100,errn_std*100,errImp_std*100,err*100,err_n*100,(*delta)*57.3,
                    errImp_max*100,errImp_upper*100,imp_max*100,curv*3.6,car_state(1)*100,errImp*100,over_flag,shift_f,dpsi*57.3,dimpsi*57.3,rol*57.3,imp_rol*57.3,err_std*100,five_flag,shift_flag,slip_out_flag,
                    slip_out_trick,slip_in_flag,slip_in_trick,across_flag,car_flag,car_cmt,ago_L,shift_cnt,shift_value*100, shift_dis*100,min_val*100,shift_f,Pitch*57.3/100,PitchIm*57.3/100,errImp_out*100,q1,q2,r,
                    car_K1,car_K2,dimer[0]*100,dimer[1]*100,down_pos,dimer[down_pos]*100,errImp_dis*100,slip_out_cnt,slip_out_cmt, Imp_out_cnt,cross_in_num*100,back_stay_cnt,back_cnt,MaxSteeringAngle,
                    off_shift_std,go_over_ratio,go_over_max,go_cross,go_cross_flag,go_ratio,go_cnt,go_flag,go_stay_cnt,go_imp_mean*100,l1,l2,l3,Aa,Bb,Cc,curx,cury,impx,
                    impy,T_psi,cur_psi,imp_psi,mini_vel_limit*3.6,Omiga,IntegralSwitchIm,GainSteeringIm_mode,GainxTrackIm_mode, GainxHeadingIm_mode,GainRIm_mode, GainLearnIm/1000,
                    OnlineAggresivenessIm_mode,ApproachAggresivenessIm_mode,PTimeOffIm/10,PTimeIm/10);
        }
        save_RQ_flag_ago = save_RQ_flag;
        CheckFileOver(fpControl, 50);
    }

    // 存储上一时刻农机与农具的状态信息
    *RQ_ago = *delta;
    if(car_online == 1 && (across_flag ==1 || slip_out_flag == 1) && over_flag == 1){
        *RQ_ago = *delta + car_K2 * pre_shoot * errImp / abs(errImp);
    }
    shift_std_ago = shift_std;
    errn_std_ago = errn_std;
    car_flag_ago = car_flag;
    cur_ago = {curx, cury, cur_psi, curv, rol, dpsi, err};
    imp_ago = {impx, impy, imp_psi, impv, imp_rol, dimpsi, errImp};

    if(MotorMoment == 99 && LogSwitch != 1){
        // 用于解决算法运行且日志存储期间，日志文件被删除后，不重启平板的情况下无法重新创建日志文件
        if (file_size_now == 0) {
            file_size_cmt++;
            if (file_size_cmt >= 8) {         // 累积计数2~5,刷新日志存储大小。精度4KB 刷新受限
                fpControl = nullptr;
                file_size_cmt = 0;
            }
        }
        // LOGI("file_size_now:%ld,file_size_cmt:%ld",file_size_now, file_size_cmt);
        if (file_size_now > 0) {
            file_size_cmt = 0;
        }
        if (save_RQ_flag % 3 == 1) {
            if (fpControl == nullptr) {
                fpControl = fopen(impControl, "wt");  // 首次调用会创建.txt文本，之后将不再创建
            }
        }
        if (save_RQ_flag == 2) {
            if (fpControl == nullptr) {
                fpControl = fopen(impControl_1, "wt");  // 首次调用会创建.txt文本，之后将不再创建
            }
        }
        if (save_RQ_flag == 3) {
            if (fpControl == nullptr) {
                fpControl = fopen(impControl_2, "wt");  // 首次调用会创建.txt文本，之后将不再创建
            }
        }
        if (save_RQ_flag != save_RQ_flag_ago) {                 // 文件达到设定的存储上限时触发
            save_RQ_flag_ago = save_RQ_flag;
            save_RQ_flag = save_RQ_flag - 1;
            CheckFileOver(fpControl, 50);                // 第一轮数据填入时，下一个文件的数据量<FileSize
            if (save_RQ_flag != save_RQ_flag_ago) {             // 处理第一轮周期 数据存入逻辑缺陷
                save_RQ_flag = save_RQ_flag_ago;
            }
            if (save_RQ_flag == 4) {
                save_RQ_flag = 1;
                save_RQ_flag_ago = 1;
            }
        }
    }
    if(LogSwitch == 1){
        if(go_line != 3){
            kylog_i("Imp_RQ_Control","Date:%04d-%02d-%02d.%02d-%02d-%02d.%03d,Week:%d,Second:%f,"
                              "%d, ,%d, ,%d, ,%d, ,%f, ,%d, ,%d, ,%f, ,%d, ,%d, ,%f, ,%f, ,%f, ,%d, ,%d, ,"
                              "%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,"
                              "%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%d, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%d, ,%d, ,%d, ,%d, ,%d, ,%d, ,"
                              "%d, ,%d, ,%d, ,%d, ,%d, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,"
                              "%f, ,%f, ,%f, ,%f, ,%d, ,%f, ,%f, ,%d, ,%d, ,%d, ,%f, ,%d, ,%d, ,%d, ,"
                              "%d, ,%f, ,%f, ,%f, ,%d, ,%f, ,%d, ,%d, ,%d, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,"
                              "%f, ,%f, ,%f, ,%f, ,%f, ,%f, ,%d, ,%d, ,%d, ,%d, ,%d, ,%d, ,"
                              "%d, ,%d, ,%d, ,%d\n",
                    bj_time.tm_year + 1900, bj_time.tm_mon + 1, bj_time.tm_mday, bj_time.tm_hour,bj_time.tm_min, bj_time.tm_sec, (int) (milliseconds * 1000), Week, Second / 100.0,
                    DriverStatus, ImplementType, imp_mode, Control_Type, impv * 3.6, car_shift, car_online, err_eso*100, delay, err_pre,shift_R,shift_R1, shift_R2,shift_cout,target_cout,
                    go_overshoot*100,pre_shoot*100,overshoot*100,dpsi_ratio,dpsi_back,car_U*57.3,delta_LQR*57.3,(*delta)*57.3,delta_imp*57.3,shift_std_ago*100,shift_std*100,errn_std*100,errImp_std*100,err*100,err_n*100,(*delta)*57.3,
                    errImp_max*100,errImp_upper*100,imp_max*100,curv*3.6,car_state(1)*100,errImp*100,over_flag,shift_f,dpsi*57.3,dimpsi*57.3,rol*57.3,imp_rol*57.3,err_std*100,five_flag,shift_flag,slip_out_flag,slip_out_trick,slip_in_flag,slip_in_trick,
                    across_flag,car_flag,car_cmt,ago_L,shift_cnt,shift_value*100, shift_dis*100,min_val*100,shift_f,Pitch*57.3/100,PitchIm*57.3/100,errImp_max*100,q1,q2,r,
                    car_K1,car_K2,dimer[0]*100,dimer[1]*100,down_pos,dimer[down_pos]*100,errImp_dis*100,slip_out_cnt,slip_out_cmt, Imp_out_cnt,cross_in_num*100,back_stay_cnt,back_cnt,MaxSteeringAngle,
                    off_shift_std,go_over_ratio,go_over_max,go_cross,go_cross_flag,go_ratio,go_cnt,go_flag,go_stay_cnt,go_imp_mean*100,l1,l2,l3,Aa,Bb,Cc,curx,cury,impx,
                    impy,T_psi,cur_psi,imp_psi,mini_vel_limit*3.6,Omiga,IntegralSwitchIm,GainSteeringIm_mode,GainxTrackIm_mode, GainxHeadingIm_mode,GainRIm_mode, GainLearnIm/1000,
                    OnlineAggresivenessIm_mode,ApproachAggresivenessIm_mode,PTimeOffIm/10,PTimeIm/10);
        }
        if(go_line == 3){
            kylog_i("Imp_RQ_Control","Date:%04d-%02d-%02d.%02d-%02d-%02d.%03d,Week:%d,Second:%f,"
                              "DriverStatus:%d,ImplementType:%d,imp_mode:%d,Control_Type:%d,impv:%f,car_shift:%d,car_online:%d,err_eso:%f,delay:%d,err_pre:%d,shift_R:%f,shift_R1:%f,shift_R2:%f,shift_cout:%d,target_cout:%d,"
                              "go_overshoot:%f,pre_shoot:%f,overshoot:%f,dpsi_ratio:%f,dpsi_back:%f,car_U:%f,delta_LQR:%f,delta:%f,delta_imp:%f,shift_std_ago:%f,shift_std:%f,errn_std:%f,errImp_std:%f,err:%f,err_n:%f,delta:%f,"
                              "errImp_max:%f,errImp_upper:%f,imp_max:%f,curv:%f,car_state(1):%f,errImp:%f,over_flag:%d,shift_f:%f,dpsi:%f,dimpsi:%f,rol:%f,imp_rol:%f,err_std:%f,five_flag:%d,shift_flag:%d,slip_out_flag:%d,"
                              "slip_out_trick:%d,slip_in_flag:%d,slip_in_trick:%d,across_flag:%d,car_flag:%d,car_cmt:%d,ago_L:%d,shift_cnt:%d,shift_value:%f,shift_dis:%f,min_val:%f,shift_f:%f,Pitch:%f,PitchIm:%f,errImp_out:%f,q1:%f,q2:%f,r:%f,"
                              "car_K1:%f,car_K2:%f,dimer[0]:%f,dimer[1]:%f,dimer[%d]:%f,errImp_dis:%f,slip_out_cnt:%d,slip_out_cmt:%d,Imp_out_cnt:%d,cross_in_num:%f,back_stay_cnt:%d,back_cnt:%d,MaxSteeringAngle:%d,"
                              "off_shift_std:%d,go_over_ratio:%f,go_over_max:%f,go_cross:%f,go_cross_flag:%d,go_ratio:%f,go_cnt:%d,go_flag:%d,go_stay_cnt:%d,go_imp_mean:%f,l1:%f,l2:%f,l3:%f,Aa:%f,Bb:%f,Cc:%f,curx:%f,cury:%f,impx:%f,"
                              "impy:%f,T_psi:%f,cur_psi:%f,imp_psi:%f,mini_vel_limit:%f,Omiga:%f,IntegralSwitchIm:%d,GainSteeringIm_mode:%d,GainxTrackIm_mode:%d,GainxHeadingIm_mode:%d,GainRIm_mode:%d,GainLearnIm:%d,"
                              "OnlineAggresivenessIm_mode:%d,ApproachAggresivenessIm_mode:%d,PTimeOffIm:%d,PTimeIm:%d\n",
                    bj_time.tm_year + 1900, bj_time.tm_mon + 1, bj_time.tm_mday, bj_time.tm_hour,bj_time.tm_min, bj_time.tm_sec, (int) (milliseconds * 1000), Week, Second / 100.0,
                    DriverStatus, ImplementType, imp_mode, Control_Type, impv * 3.6, car_shift, car_online, err_eso*100, delay, err_pre,shift_R,shift_R1, shift_R2,shift_cout,target_cout,
                    go_overshoot*100,pre_shoot*100,overshoot*100,dpsi_ratio,dpsi_back,car_U*57.3,delta_LQR*57.3,(*delta)*57.3,delta_imp*57.3,shift_std_ago*100,shift_std*100,errn_std*100,errImp_std*100,err*100,err_n*100,(*delta)*57.3,
                    errImp_max*100,errImp_upper*100,imp_max*100,curv*3.6,car_state(1)*100,errImp*100,over_flag,shift_f,dpsi*57.3,dimpsi*57.3,rol*57.3,imp_rol*57.3,err_std*100,five_flag,shift_flag,slip_out_flag,
                    slip_out_trick,slip_in_flag,slip_in_trick,across_flag,car_flag,car_cmt,ago_L,shift_cnt,shift_value*100, shift_dis*100,min_val*100,shift_f,Pitch*57.3/100,PitchIm*57.3/100,errImp_out*100,q1,q2,r,
                    car_K1,car_K2,dimer[0]*100,dimer[1]*100,down_pos,dimer[down_pos]*100,errImp_dis*100,slip_out_cnt,slip_out_cmt, Imp_out_cnt,cross_in_num*100,back_stay_cnt,back_cnt,MaxSteeringAngle,
                    off_shift_std,go_over_ratio,go_over_max,go_cross,go_cross_flag,go_ratio,go_cnt,go_flag,go_stay_cnt,go_imp_mean*100,l1,l2,l3,Aa,Bb,Cc,curx,cury,impx,
                    impy,T_psi,cur_psi,imp_psi,mini_vel_limit*3.6,Omiga,IntegralSwitchIm,GainSteeringIm_mode,GainxTrackIm_mode, GainxHeadingIm_mode,GainRIm_mode, GainLearnIm/1000,
                    OnlineAggresivenessIm_mode,ApproachAggresivenessIm_mode,PTimeOffIm/10,PTimeIm/10);
        }
    }
    if(Control_Type == 2 || (imp_mode != 22 && ((car_shift == 0 && curv > 0) || (curv < 0 && time_on == 2)))){
        err_eso = 0;
    }
    return err_eso;
}