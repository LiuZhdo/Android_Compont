//
// Created by admin on 2024/11/28.
//
#include "include/Agguide.h"
#include "include/AgguideIm.h"
#include "include/android_log.h"
#include <sys/stat.h>
#include <unistd.h>
#include <cmath>
#include "data_reconstruct.h"
#include "Imp_Ins_Opt.h"

// 检查指定文件的大小，如果文件大小超过给定的上限，则清空文件内容并重置文件指针
void CheckDataFileOver(FILE *fp, int FileSizeMB)  // FileSizeMB 文件大小的上限（以 MB 为单位）
{
    int s_log_file_fd = -1;                       // 存储文件描述符，初始值为 -1
    struct stat logFileStat = {0};                // 存储文件的状态信息
    // 使用 fileno 函数从文件指针 fp 中获取文件描述符，并将其赋值给 s_log_file_fd
    s_log_file_fd = fileno(fp);
    // 使用 fstat 函数获取文件描述符 s_log_file_fd 对应的文件的状态信息，并存储在 logFileStat 结构体
    fstat(s_log_file_fd, &logFileStat);
    // 检查文件大小
    if (((logFileStat.st_size * 1.0) / (1024 * 1024)) > FileSizeMB)
    {
        // 使用 ftruncate 函数将文件大小截断为零，即清空文件内容
        ftruncate(s_log_file_fd, 0);  /* truncate the log file to empty file */
        // 使用 lseek 函数将文件指针移动到文件的开头
        lseek(s_log_file_fd, 0, SEEK_SET);
    }
}

// 计算平均值与方差
void calculateSD(double data[], int len , double *SD_value, double *mean_value)
{
    double sum = 0.0, mean, standardDeviation = 0.0;
    for(int i = 0; i < len; ++i)
    {
        sum += data[i];
    }
    mean = sum / (1.0 * len);                           // 计算算术平均值 浮点数转换
    for(int i = 0; i < len; ++i)
    {
        standardDeviation += pow(data[i] - mean, 2);
    }
    // 后期阈值的选取也是依据下式计算的结果。标准差放大100倍，方便比较观察数据。
    *SD_value = sqrt(standardDeviation / 1.0*len);   // 计算标准差 浮点数转换
    *mean_value = mean;
}

// 通过高度差，判定农具是否放下 imp_arm_z为系统输入的农具天线高度(农具放下时测量) current_imp_z为当前在投影转换后农具控制点的高度
bool ImplementYarmCorrect(double imp_arm_z, double tractor_arm_z, double current_imp_z, double current_tractor_z)
{
    double diff = 0;
    double diff_arm = tractor_arm_z - imp_arm_z;
    // 如果农具正常放下，那么农机与农具的投影的控制点应当在同一平面，即z坐标基本一致.
    double real_diff = current_tractor_z - current_imp_z;
    double temp = imp_arm_z;                                   // 存储农具杆臂值 输入的系统参数，未投影前的农具实际高度
    const int len = 20;
    static double real_diff_buf[len] = {0.0};              // 存储投影高度差值
    int count = 0;
    for(int i = 0; i < len; i++)
    {
        if(i == (len-1))
        {
            real_diff_buf[i] = real_diff;
        }
        else
        {
            real_diff_buf[i] = real_diff_buf[i+1];
        }
        // 20cm差值范围判定，抬起时则基本在-0.5m的偏差。
        // 如果投影高度差值小于-0.2(含符号比较)，计数+1 农具处于抬起状态，
        if(real_diff_buf[i] < -0.3)
        {
            count++;
        }
    }
    // 计数超过len-2  较长时间处于抬起状态
    if(count >(len-2)) {
        temp = imp_arm_z - fabs(real_diff);
    }
//    LOGI("ImplementYarmCorrect: diff_arm=%.3f real_diff=%.3f temp=%.3f diff=%.3f tractor_arm_z=%.3f imp_arm_z=%.3f current_tractor_z=%.3f current_imp_z=%.3f\n",
//         diff_arm, real_diff, temp, diff, tractor_arm_z, imp_arm_z, current_tractor_z, current_imp_z);
    LOGI("imudata_paraimplementYarmCorrect: diff_arm=%.3f real_diff=%.3f temp=%.3f diff=%.3f,real_diff_buf[0]=%.3f,real_diff_buf[1]=%.3f,"
         "real_diff_buf[2]=%.3f,tractor_arm_z=%.3f imp_arm_z=%.3f current_tractor_z=%.3f current_imp_z=%.3f\n",
         diff_arm, real_diff, temp, diff, real_diff_buf[0],real_diff_buf[1],
         real_diff_buf[2],tractor_arm_z, imp_arm_z, current_tractor_z, current_imp_z);
    if(temp != imp_arm_z)
    {
        return true;
    }
    else
    {
        return false;
    }
    //return temp;
}
// 长度为1024的字符数组，用于存储文件路径和名称
char TowImplementControlIMUcalDataName[1024] = "/sdcard/ControllerX/ImpImuCal.txt";
// FILE 是C语言中用于表示文件的结构体类型 定义静态文件指针 在程序中管理与 IMU 校准数据记录相关的文件操作
static FILE *fpIMUCalibration = NULL;
/**
 *
 * @param auto_drive
 * @param calibration_cmd_imp  校准阶段标识词  11=开始校准，12=第一次采集开始，16=第二次采集开始，20=结束校准
 * @param tractor_speed
 * @param e
 * @param n
 * @param u_imp
 * @param u_tractor
 * @param xtrack
 * @param xhead
 * @param line_a
 * @param line_b
 * @param line_c
 * @param roll
 * @param pitch
 * @param yaw
 * @param arm_imp_x         系统参数 输入的农具天线的位置
 * @param arm_imp_y
 * @param arm_imp_z
 * @param arm_tractor_z
 * @param out_PitchOffsetIm
 * @param out_RollOffsetIm
 * @param out_HardwareStatus
 * @param out_CalibratePhaseIm 校准结果标识词 21=开始校准，22=第一次采集成功，23=第二次成功，29=校准失败，30=校准成功
 * @param out_ArmXIm
 */
double roll_SD = 0, pitch_SD = 0, mean_roll = 0, mean_pitch = 0;
extern "C" {
void ImplementImuInstallationCalibration(   // 50ms
        double auto_drive,
        double calibration_cmd_imp,
        double tractor_speed,
        double e,
        double n,
        double u_imp,       // m
        double u_tractor,   // m
        double xtrack,
        double xhead,
        double line_a,
        double line_b,
        double line_c,
        double roll,
        double pitch,
        double yaw,
        double arm_imp_x,
        double arm_imp_y,
        double arm_imp_z,                            // m
        double arm_tractor_z,                        // m
        short *out_PitchOffsetIm,                    // 0.01degree
        short *out_RollOffsetIm,
        long long int *out_HardwareStatus,          // 硬件状态
        unsigned short *out_CalibratePhaseIm,       //校准结果
        short *out_ArmXIm)
{
    const int check_len = 150;                          // 数据采集个数  原算法100
    static double roll_buf_1[check_len] = {0.0};    // 第一躺roll
    static double pitch_buf_1[check_len] = {0.0};   // 第一趟pitch
    static double roll_buf_2[check_len] = {0.0};    // 第二趟roll
    static double pitch_buf_2[check_len] = {0.0};   // 第二趟pitch
    static double roll_check[5] = {0.0};            // roll下发错误检查
    static double pitch_check[5] = {0.0};           // pitch下发错误检查
    static double xtrack_check[5] = {0.0};          // 横向偏差检查
    static double xhead_check[5] = {0.0};           // 航向偏差检查
    static double mean_roll_1 = 0;                      // 第一趟roll平均值
    static double mean_roll_2 = 0;
    static double mean_pitch_1 = 0;                     // 第一趟pitch平均值
    static double mean_pitch_2 = 0;
    static double roll_sd[2] = {0.0};               // 方差记录
    static int calibration_succ_fail_whole = 0;         // 错误码
    static int calibration_succ_fail_part = 0;          // 1 = 首次采集成功， 2 = 二次采集成功
    int res = 0;                                        // 1 = success, 2= failure
    static int counter_collect_data = 0;                // 采集数据次数计数
    static int label = 0;                               // 记录日志用
    double arm_diff = arm_tractor_z - arm_imp_z;
    double u_diff = u_tractor - u_imp;
    LOGI("roll=%.3f pitch=%.3f tractor_arm_z=%.3f imp_arm_z=%.3f arm_diff=%.3f current_tractor_z=%.3f current_imp_z=%.3f u_diff=%.3f",roll,pitch,arm_tractor_z, arm_imp_z, arm_diff, u_tractor, u_imp,u_diff);
    if (calibration_cmd_imp == 11)                      // 校准开始
    {
        *out_CalibratePhaseIm = 21;                     // 校准阶段标志词
        *out_PitchOffsetIm = 0;                         // 俯仰角校准结果
        *out_RollOffsetIm = 0;                          // 横滚角校准结果
        counter_collect_data = 0;
        mean_roll_1 = 0;
        mean_roll_2 = 0;
        mean_pitch_1 = 0;
        mean_pitch_2 = 0;
        calibration_succ_fail_whole = 0;
        calibration_succ_fail_part = 0;
        roll_sd[0] = 0;
        roll_sd[1] = 0;
        // 59~61位为标志词
        HardwareStatus &= ~(((long long int)1) << 59);
        HardwareStatus &= ~(((long long int)1) << 60);
        HardwareStatus &= ~(((long long int)1) << 61);
        if (fpIMUCalibration == NULL)       // 如果为 NULL，则表示文件指针没有被打开
        {
            fpIMUCalibration = fopen(TowImplementControlIMUcalDataName,"a+");
            setbuf(fpIMUCalibration,NULL);
            // fprintf(fpIMUCalibration, "校准数据: %f\n", calibrationData);

        }
        label = 0;                      // 开启记录日志阶段的标记量
    }
    // 点击开始采集按钮后，进入第一次采集阶段
    else if (calibration_cmd_imp == 12 && calibration_succ_fail_part == 0 && calibration_succ_fail_whole == 0)  // 第一次开始采集
    {
        counter_collect_data++;
        int count_check_roll = 0;
        int count_check_pitch = 0;
        int count_check_xhead = 0;
        int count_check_xtrack = 0;
        for (int i = 0; i < 5; i++)     // 检查姿态角是否异常
        {
            if (i == 4) {
                roll_check[i] = roll;       // 记录横滚角
                pitch_check[i] = pitch;     // 记录俯仰角
                xtrack_check[i] = xtrack;   // 记录横偏误差
                xhead_check[i] = xhead;     // 记录航偏误差
            } else {
                pitch_check[i] = pitch_check[i + 1];
                roll_check[i] = roll_check[i + 1];
                xtrack_check[i] = xtrack_check[i + 1];
                xhead_check[i] = xtrack_check[i + 1];
            }
            if (counter_collect_data > 10) {  // 采集数据超过10个后
                // 浮点数判断是否为0，如果为0，则说明横滚与俯仰角未传入(正确)数值 未下发数值
                if (fabs(roll_check[i]) < 0.000001) {
                    count_check_roll++;
                }
                if (fabs(pitch_check[i]) < 0.000001) {
                    count_check_pitch++;
                }
                // 校准导航线的偏差应当在合理的范围内  原来算法横偏0.4,航偏15°
                if (fabs(xtrack_check[i]) > 0.2) {
                    count_check_xtrack++;
                }
                // 通过行驶的距离与农机的横偏限制车辆所在的位置，位置是关键因素
                if (fabs(xhead_check[i]) > 10 * 3.14159 / 180) {
                    count_check_xhead++;
                }
            }
        }
        if (count_check_pitch == 5 || count_check_roll == 5)
        {
            calibration_succ_fail_whole = 1;  // 姿态角数据有误，生成错误码
        }
        if (count_check_xhead == 5 || count_check_xtrack == 5)
        {
            calibration_succ_fail_whole = 2;  // 采集数据点处位置偏差过大，生成错误码
        }
        if (ImplementYarmCorrect(arm_imp_z, arm_tractor_z, u_imp, u_tractor))
        {
            calibration_succ_fail_whole = 3;  // 采集时农具未放下，生成错误码
            //*out_CalibratePhaseIm = 13;
            counter_collect_data = 0;         // 检查是否放下农具，直至放下重新初始化；农具未放下则提示后重新采集。
        }
        for (int j = 0; j < check_len; j++)  // 填充数据采集器  check_len=100
        {
            if (j == check_len - 1)
            {
                roll_buf_1[j] = roll;
                pitch_buf_1[j] = pitch;
            }
            else
            {
                roll_buf_1[j] = roll_buf_1[j + 1];
                pitch_buf_1[j] = pitch_buf_1[j + 1];
            }
        }
        // double roll_SD = 0, pitch_SD = 0, mean_roll = 0, mean_pitch = 0;
        // 对局部变量取地址符号会导致NX510与NX600软件崩溃
        if (counter_collect_data > (check_len+20))  // 数据采集器填充完毕 采集20个数据点
        {
            LOGI("first_collect_data");
/*            calculateSD(roll_buf_1, check_len, &roll_SD, &mean_roll);
            calculateSD(pitch_buf_1, check_len, &pitch_SD, &mean_pitch);*/
            Reconstruct(roll_buf_1, check_len, &roll_SD, &mean_roll);
            Reconstruct(pitch_buf_1, check_len, &pitch_SD, &mean_pitch);
            // LOGI("imudata_paraImplementImuInstallationCalibration: roll_SD =%.3f mean_roll = %.3f\n", roll_SD*180/3.14159, mean_roll*180/3.14159);
            LOGI("imudata_paraimp_1: roll_SD = %.3f,mean_roll = %.3f\n", roll_SD*180/3.14159, mean_roll*180/3.14159);
            LOGI("imudata_paraimp_1: pitch_SD = %.3f,mean_pitch = %.3f\n", pitch_SD*180/3.14159, mean_pitch*180/3.14159);
            if (roll_SD < 10 * 3.14159 / 180.0)  // 0.02617 如果横滚角标准差小于0.1° 弧度制比较
            {
                calibration_succ_fail_part = 1;     // 第一次采集成功
                mean_roll_1 = mean_roll;
                mean_pitch_1 = mean_pitch;
                counter_collect_data = 0;           // 重置数据采集计数
                *out_CalibratePhaseIm = 22;         // 标志词
                roll_sd[0] = roll_SD*180/3.14159;   // 转换为度数
            }
            else
            {
                if (counter_collect_data >= 350)  // 采集数据超过15秒，仍然不稳定  原数据300
                {
                    calibration_succ_fail_whole = 4;  // 姿态数据不稳定， 第一次采集失败，生成错误码
                    counter_collect_data = 0;
                }
            }
        }
        LOGI("imudata_para_first:%.2f, %d, %.6f, %.6f, %d, %d, %.3f, %.3f, %.3f, %.3f\n",Second*0.01,counter_collect_data,
             roll*180/M_PI,pitch*180/M_PI,calibration_succ_fail_whole,calibration_succ_fail_part,roll_SD,mean_roll*180/M_PI,pitch_SD,mean_pitch*180/M_PI);
        if (fpIMUCalibration != NULL)   // Second数值之间间隔0.10
        {
            fprintf(fpIMUCalibration, "sector1: %.2f, %d, %.6f, %.6f, %d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                    Second*0.01,
                    counter_collect_data,
                    roll*180/M_PI,           // 角度制
                    pitch*180/M_PI,
                    calibration_succ_fail_whole,
                    calibration_succ_fail_part,
                    roll_SD,
                    mean_roll,
                    pitch_SD,
                    mean_pitch,
                    arm_imp_z,
                    arm_tractor_z,
                    u_imp,
                    u_tractor);
            fflush(fpIMUCalibration);
        }
    }
    else if (calibration_cmd_imp == 16 && calibration_succ_fail_part == 1 && calibration_succ_fail_whole == 0 )  // 第二次开始采集，第一次采集必须成功
    {
        counter_collect_data++;
        int count_check_roll = 0;
        int count_check_pitch = 0;
        int count_check_xhead = 0;
        int count_check_xtrack = 0;
        for (int i = 0; i < 5; i++)  // 检查姿态角是否异常
        {
            if (i == 4)
            {
                roll_check[i] = roll;
                pitch_check[i] = pitch;
                xtrack_check[i] = xtrack;
                xhead_check[i] = xhead;
            }
            else
            {
                pitch_check[i] = pitch_check[i + 1];
                roll_check[i] = roll_check[i + 1];
                xtrack_check[i] = xtrack_check[i + 1];
                xhead_check[i] = xhead_check[i + 1];
            }
            if (counter_collect_data > 10)
            {
                if (fabs(roll_check[i]) < 0.000001)
                {
                    count_check_roll++;
                }
                if (fabs(pitch_check[i]) < 0.000001)
                {
                    count_check_pitch++;
                }
                if (fabs(xtrack_check[i]) > 0.2)
                {
                    count_check_xtrack++;
                }
                if (fabs(xhead_check[i]) > 10 * 3.14159 / 180)
                {
                    count_check_xhead++;
                }
            }
        }
        if (count_check_pitch == 5 || count_check_roll == 5)
        {
            calibration_succ_fail_whole = 1;  // 误姿态角数据，生成错误码
        }
        if (count_check_xhead == 5 || count_check_xtrack == 5)
        {
            calibration_succ_fail_whole = 2;  // 采集数据点处偏差过大，生成错误码
        }
        if (ImplementYarmCorrect(arm_imp_z, arm_tractor_z, u_imp, u_tractor))
        {
            calibration_succ_fail_whole = 3;  // 采集时农具未放下，生成错误码
            //*out_CalibratePhaseIm = 17;
        }
        for (int j = 0; j < check_len; j++)  // 填充数据采集器
        {
            if (j == check_len - 1)
            {
                roll_buf_2[j] = roll;
                pitch_buf_2[j] = pitch;
            }
            else
            {
                roll_buf_2[j] = roll_buf_2[j + 1];
                pitch_buf_2[j] = pitch_buf_2[j + 1];
            }
        }

        if (counter_collect_data > (check_len+20))  // 数据采集器填充完毕
        {
            LOGI("second_collect_data");
/*            calculateSD(roll_buf_2, check_len, &roll_SD, &mean_roll);
            calculateSD(pitch_buf_2, check_len, &pitch_SD, &mean_pitch);*/
            Reconstruct(roll_buf_2, check_len, &roll_SD, &mean_roll);
            Reconstruct(pitch_buf_2, check_len, &pitch_SD, &mean_pitch);
            // LOGI("IMudata_paraimplementImuInstallationCalibration: roll_SD =%.3f mean_roll = %.3f\n", roll_SD*180/3.14159, mean_roll*180/3.14159);
            LOGI("imudata_paraimp_2: roll_SD =%.3f mean_roll = %.3f\n", roll_SD*180/3.14159, mean_roll*180/3.14159);
            LOGI("imudata_paraimp_2: pitch_SD = %.3f,mean_pitch = %.3f\n", pitch_SD*180/3.14159, mean_pitch*180/3.14159);
            if (roll_SD < 10 * 3.14159 / 180.0)
            {
                calibration_succ_fail_part = 2;  // 第二次采集成功
                mean_roll_2 = mean_roll;
                mean_pitch_2 = mean_pitch;
                counter_collect_data = 0;
                *out_CalibratePhaseIm = 23;
                roll_sd[1] = roll_SD*180/3.14159;
            }
            else
            {
                if (counter_collect_data >= 350 )  // 采集数据超过15秒，仍然不稳定 原数据300
                {
                    calibration_succ_fail_whole = 4;  // 姿态数据不稳定， 第二次采集失败，生成错误码
                    counter_collect_data = 0;
                }
            }
        }
        LOGI("imudata_para_second:%.2f, %d, %.6f, %.6f, %d, %d, %.3f, %.3f, %.3f, %.3f\n",Second*0.01,counter_collect_data,
             roll*180/M_PI,pitch*180/M_PI,calibration_succ_fail_whole,calibration_succ_fail_part,roll_SD,mean_roll_2*180/M_PI,pitch_SD,mean_pitch_2*180/M_PI);
        if (fpIMUCalibration != NULL)
        {
            fprintf(fpIMUCalibration, "sector2: %.2f, %d, %.6f, %.6f, %d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                    Second*0.01,
                    counter_collect_data,
                    roll*180/M_PI,       // 角度制
                    pitch*180/M_PI,
                    calibration_succ_fail_whole,
                    calibration_succ_fail_part,
                    roll_SD,
                    mean_roll,
                    pitch_SD,
                    mean_pitch,
                    arm_imp_z,
                    arm_tractor_z,
                    u_imp,
                    u_tractor);
            fflush(fpIMUCalibration);
        }
    }
    else if (calibration_cmd_imp == 20 || calibration_succ_fail_whole != 0)// || calibration_succ_fail_whole != 0)  // 结束校准
    {
        if (calibration_succ_fail_whole == 0 && calibration_succ_fail_part == 2)  // 无错误码且二次采集成功
        {
            double temp1 = 0;
            temp1 = (short) ((0.5 * mean_roll_1 + 0.5 * mean_roll_2) * 100 * 180 / M_PI);
            if(fabs(temp1 * 0.01) > 10.0)  // 结果较大  校准失败
            {
                res = 2;
                *out_CalibratePhaseIm = 29;
                counter_collect_data = 0;           // 重置标志计数器
                calibration_succ_fail_whole = 4;
            }
            else
            {
                res = 1;                                           // 校准成功
                *out_RollOffsetIm = (short) ((0.5 * mean_roll_1 + 0.5 * mean_roll_2) * 100 * 180 / M_PI);
                *out_PitchOffsetIm = (short) ((0.5 * mean_pitch_1 + 0.5 * mean_pitch_2) * 100 *180 / M_PI);
                *out_CalibratePhaseIm = 30;                        // 校准成功标志词
                HardwareStatus &= ~(((long long int)1) << 59);     // 硬件状态设置
                HardwareStatus &= ~(((long long int)1) << 60);
                HardwareStatus &= ~(((long long int)1) << 61);
            }
        }
        else
        {
            res = 2;
            *out_CalibratePhaseIm = 29;
            counter_collect_data = 0;
        }
        if (res == 2)  // 反馈错误码  1<<59, 60, 61
        {
            if (calibration_succ_fail_whole == 1)  // 姿态角为0
            {
                HardwareStatus |= ((long long int)1) << 61;
            }
            else if (calibration_succ_fail_whole == 2)  // 采集点处偏差大
            {
                HardwareStatus |= ((long long int)1) << 59;
            }
            else if (calibration_succ_fail_whole == 3)  // 农具未放下
            {
                HardwareStatus |= ((long long int)1) << 60;
            }
            else if (calibration_succ_fail_whole == 4)  // 姿态波动大
            {
                HardwareStatus |= ((long long int)1) << 61;
            }
        }
        LOGI("imudata_paraimp_final:%lld,%.3f, %d, %d, %d, mean_roll_1:%.3f, mean_roll_2:%.3f, roll_sd[0]:%.3f, "
             "roll_sd[1]:%.3f, mean_pitch_1:%.3f, mean_pitch_2:%.3f, ins_roll:%d, ins_pitch:%d\n",HardwareStatus,Second*0.01,calibration_succ_fail_whole,
             calibration_succ_fail_part,res,mean_roll_1*180/M_PI,mean_roll_2*180/M_PI,roll_sd[0],roll_sd[1],
             mean_pitch_1*180/M_PI,mean_pitch_2*180/M_PI,*out_RollOffsetIm,*out_PitchOffsetIm);
        if(fpIMUCalibration != NULL && label < 3)
        {
            label++;
            fprintf(fpIMUCalibration,"sector3: %.3f, %d, %d, %d, %.3f, %.3f, %.3f, %.3f, %d, %d, %.3f, %.3f, %.3f, %.3f\n",
                    Second*0.01,
                    calibration_succ_fail_whole,
                    calibration_succ_fail_part,
                    res,
                    mean_roll_1,
                    mean_roll_2,
                    roll_sd[0],
                    roll_sd[1],
                    *out_PitchOffsetIm,
                    *out_RollOffsetIm,
                    arm_imp_z,
                    arm_tractor_z,
                    u_imp,
                    u_tractor);
            fflush(fpIMUCalibration);
            CheckDataFileOver(fpIMUCalibration, 100);
        }
        else
        {
            if(fpIMUCalibration != NULL)
            {
                fclose(fpIMUCalibration);
                fpIMUCalibration = NULL;
                CalibrateCmdIm = 0;
            }

        }
    }
    else
    {
        //return;
    }
    LOGI("imudata_paraimplementImuInstallationCalibration:calibration_succ_fail_whole=%d calibration_succ_fail_part=%d calibration_cmd_imp=%.1f  *out_CalibratePhaseIm=%d res=%d,"
         "counter_collect_data=%d, mean_roll_1=%.3f mean_roll_2=%.3f roll_sd[0]=%.3f roll_sd[1]=%.3f *out_PitchOffsetIm=%d *out_RollOffsetIm=%d,arm_imp_x=%.3f,arm_imp_y=%.3f,arm_imp_z=%.3f,out_ArmXIm=%d",
         calibration_succ_fail_whole, calibration_succ_fail_part, calibration_cmd_imp, *out_CalibratePhaseIm, res, counter_collect_data,
         mean_roll_1*180/M_PI,mean_roll_2*180/M_PI, roll_sd[0], roll_sd[1],*out_PitchOffsetIm,*out_RollOffsetIm,arm_imp_x,arm_imp_y,arm_imp_z,*out_ArmXIm);
}
}