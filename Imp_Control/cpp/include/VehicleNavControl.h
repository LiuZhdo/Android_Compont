/*
 * VehicleNavControl.h
 *
 *  Created on: Jul 9, 2021
 *      Author: huace
 */

#ifndef CONTROL_VEHICLENAVCONTROL_VEHICLENAVCONTROL_H_
#define CONTROL_VEHICLENAVCONTROL_VEHICLENAVCONTROL_H_

/*
1、各转向类型控制时，保持参数含义一致，且尽量适配默认值（最好是一个参数调整直线度）
2、各转向类型控制,保留模型控制、执行机构闭环控制和校准功能函数独立性
*/
typedef enum
{
    VehicleType_Front = 1,//前轮转向
    VehicleType_Rear,//后轮转向
    VehicleType_Crawer,//方向盘履带转向
    VehicleType_Articulation,//铰接转向
    VehicleType_Crawer2,//差速履带转向
}VehicleType_t;
typedef enum
{
    ControlerType_PA_2 = 1,//
    ControlerType_PA_3,
}ControlerType_t;
typedef enum
{
    GNSSModuleType_SPP = 0,
    GNSSModuleType_DGPSRTD,
    GNSSModuleType_RTK,
    GNSSModuleType_RangePointRTX,
    GNSSModuleType_CenterPointRTX,
}GNSSModuleType_t;
typedef enum
{
    AngleSensorType_Code = 1,//
    AngleSensorType_GASensor,
    AngleSensorType_NoSensor,
    AngleSensorType_NoSensor2,//无-插秧机
}AngleSensorType_t;
typedef enum
{
    ControlType_LQR = 1,//
    ControlType_MPC,
    ControlType_MPC2,//增量式
    ControlType_NX01,
    ControlType_cure,//纯追踪
}ControlType_t;
typedef enum
{
    Linetype_Straight = 1,//
    Linetype_Arc,
    Linetype_Curve,//
    Linetype_Turn,//调头
}LineType_t;

typedef struct //车辆横向控制参数输入,此结构体数据变量只能使用，不允许修改
{
    //车辆参数
    VehicleType_t     VehicleType;//车辆类型，1前轮，2后轮，3履带(方向盘)，4铰接，5履带（差分）
    ControlerType_t   ControlerType;//1:pa-2,2:Pa-3
    GNSSModuleType_t  GNSSModuleType;
    AngleSensorType_t AngleSensorType;//角度传感器类型,1机械，2：GASensor,3：无，4：无-插秧机，
    double L1;  //[A]m,前后轴距离,【铰接点-前轴距离】
    double L2;  //[B]m,后轮轴-挂载点间距，【铰接点-后轴距离】
    double L3;  // m 后轮轴高度
    double FrontHitchDistance;//[G]m,前轮轴距
    double WASMaxL;     //最大转角，rad
    double WASMaxR;     //最大转角，rad
    double SPEED_MIN;   //最小车速，m/s
    //模型控制参数
    ControlType_t  ControlType;//模型控制方法类型，
    bool   IntegralSwitch;//积分开关，true，开，false 关
    char   SlopeCmd;//滑坡类型,1:slope,2:standard
    double GainStraightness;//一个参数调整直线度,50%~150%,默认100%

    double GainSteering;//转向灵敏度

    double GainxTrack;//横向灵敏度
    double GainxHeading;//航向灵敏度
    double GainR;//控制灵敏度
    double GainLearn;//学习灵敏度，也可用作倒车参数调整

    double PTime;//在线判断时间,0.1s
    double PTimeOff;//离线判断时间,0.1s
    double OnlineAggresiveness;//在线进度: 一个参数调整多个参数
    double ApproachAggresiveness;//入线进度
    double GainB;
    double GainS;
    double L4;  // 挂载点到犁轴距离
    double L5;  // 犁高
    double L6;  // 拖拉机后轴到挂载点距离
    double vehicle_type;  // 车辆类型： 1 = 三点悬挂, 2 = 单点悬挂， 3 = 对行，备用
}VehicleNavParasIn_T;

typedef struct //车辆横向控制实时数据输入,此结构体数据变量只能使用，不允许修改,
{
    bool LogStoreEn;//是否开启数据保存，存储的数据应按照统一标准格式
    //实时观测数据
    unsigned char FixState;//固定状态，1单点，2伪距，5浮动，4差分
    double SystemTime;//s
    double E;// m
    double N;// m
    double U;//m
    double Yaw;//rad
    double Speed;//m/s
    double Omiga;//车身转速，rad/s,顺时针为正 航向变化率  需要使用  引入农机导航  待定
    double Omiga_implement; // 农具转速 rad/s
    double WAS;//前轮转角，rad 当前真实转角值 考虑使用 增强实际跟随效果 坡地
    double Roll;  // 车身横滚角
    double MotorSpeed;//rad/s，电机编码器实时速度，只对电机类型有效
    double MotorPosition;//rad,电机编码器实时位置，只对电机类型有效
    double PureTrackU;//rad,软件下发的控制量
    double Yaw_impmlement;  // rad
    double E_implement;  // m
    double N_implement;  // m
    double Roll_implement;  // 农具横滚角
    double ctr_period;  // units=second
    double Speed_true;  // 真实车速，正负
    double Speed_imp;  // 农具速度，正负
    //导航线参数
    LineType_t LineType;//1：直线、2：圆，3：曲线，4：调头或显示器控制指令
    bool   LineParasOK;
    double LineA;
    double LineB;
    double LineC;
    double LineDirAngle;//rad
    //标准曲线参数,An1,Bn1,Cn1,Dn1,Ae1,Be1,Ce1,De1,An2,Bn2,Cn2,Dn2,Ae2,Be2,Ce2,De2,sk1,sk2,sk3
    double CurveLinePara[19];

    bool AutoCmd;// true auto   false manual
    bool IsOnLineFlag;//ture:online,false:offline
    bool IsOnLineFlag_implement;  // implement: ture:online,false:offline
    bool IsOnLineFlag_implement_2;  // 微调直线度用
    //无人
    short ElecControl;//无人运动状态
}VehicleNavDatasIn_T;

typedef struct //车辆横向控制输出
{
    double ResE;// m，返回滤波，Longitude
    double ResN;// m，返回滤波，Latitude
    double ResSpeed;//m/s，返回滤波
    double ResYaw;// rad，返回滤波，GPSHeading
    double xDelta;//rad,角度偏差   degree
    double xTrack;// m,农机横向偏差
    double xTrack_implement; // m, 农具横向偏差
    double xHeading;//rad，农机航向偏差
    double xHeading_implement;  // rad. 农具航向偏差
    double xRate;//期望的前轮转速，rad/s，xRate = 0.0001；
    double DeltaAm;//期望的前轮转角,rad
    bool IsOnLineFlag;//ture:online,false:offline
    bool IsOnLineFlag_Implement;  // true = online; false = offline
    bool IsLeverArmCorrect;
    //无人
    short ElecControlStatus;//无人运动状态反馈
}VehicleNavOut_T;
/*车辆模型控制全局变量*/
extern VehicleNavParasIn_T        VehicleNavParasIn;//车辆模型控制相关参数，【不允许修改】
extern VehicleNavDatasIn_T        VehicleNavDatasIn;//车辆模型控制相关实时数据，【不允许修改】
extern VehicleNavOut_T            VehicleNavOut;//车辆模型控制输出的数据，【要修改】
#endif /* CONTROL_VEHICLENAVCONTROL_VEHICLENAVCONTROL_H_ */
