#ifndef PID_H
#define PID_H

//typedef signed char int8_t;
typedef signed short int int16_t;
//typedef signed int int32_t;
//typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
//typedef unsigned short int uint16_t;
//typedef unsigned int uint32_t;
//typedef unsigned long long uint64_t;
//typedef unsigned char bool_t;
typedef float fp32;
//typedef double fp64;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);

#endif
