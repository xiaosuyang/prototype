/*
 * @FilePath: \ES02TC\pid_fuzzy.h
 * @Author: LATCOS-Lsp
 * @Date: 2016-09-01 23:18:38
 * @LastEditTime: 2023-12-08 10:26:48
 * @LastEditors: LATCOS-Lsp
 * @Description:
 */
#ifndef fPID_H_
#define fPID_H_

#include "stdint.h"

float LinearRealization(float,float,float);

typedef enum
{
    F_DIRECT,
    F_REVERSE
}
FPIDDirection;

class fuzzypid
{
    public:
    fuzzypid()=default;
    void fuzzypid_init(float Kp,float Ki, float Ks,float sampleTimeSeconds, FPIDDirection controllerDirection_p)
    {
        kp=Kp;
        Ki=ki;
        ks=Ks;

        controllerDirection=controllerDirection_p;
        sampletime=sampleTimeSeconds;

        outMax=10;
        outMin=-10;
        PIDturningset();

    }

    void Fuzzyset(float MaxdP,float MindP,float MaxdI,float MindI,float errormax,float errormin)
    {
        maxdKp=MaxdP;
        mindKp=MindP;
        maxdKi=MaxdI;
        mindKi=MindI;

        

        Vmax=errormax;
        Vmin=errormin;
    }


    void LinearQuantization(float *qValue);

    void CalcMembership(float* ms, float qv,int * index);

    void FuzzyComputation(float *DeltaK);

    void PIDturningset();

    void PIDcompute();

    void FeedforwardAdd(bool flag);

    void setpointset(float setpointp)
    {
        lastSetpoint=setpoint;
        setpoint=setpointp;
    }

    void inputset(float input_p)
    {
        input=input_p;
    }

    FPIDDirection controllerDirection;
    

    static int ruleKp[7][7];
    static int ruleKi[7][7];

    float input;
    float setpoint;  /*设定值*/
    float lastSetpoint;

    float kp;        /*比例系数*/
    float ki;        /*积分系数*/

    float dispKp;
    float dispKi;
    float dInput;
    // 

    float alteredKp;
    float alteredKi;

    
    float ks;        /*前馈系数*/

    float error;
    float lasterror; /*前一拍偏差*/
    float preerror;  /*前两拍偏差*/
    
    float deltaerror;

    float iTerm;

    float deadband;  /*死区*/
    float output;    /*输出值*/
    float sumerror;

    float outMax;   /*输出值的上限*/
    float outMin;   /*输出值的下限*/

    float Vmax;     /*误差上限*/
    float Vmin;        /* 误差下限*/

    float maxdKp; /*Kp增量的最大限值*/
    float mindKp; /*Kp增量的最小限值*/
    float qKp;    /*Kp增量的影响系数*/
    float maxdKi; /*Ki增量的最大限值*/
    float mindKi; /*Ki增量的最小限值*/
    float qKi;    /*Ki增量的影响系数*/


    float deltaKi; 
    float deltaKp; 

    float FF;
    float desiredDelta;

    float qvalscope[2]={0};


    float sampletime;




};









#endif /* PID_H_ */