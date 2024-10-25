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

class fuzzypid
{
    public:
    fuzzypid(float Kp,float Ki, float Ks)
    {
        kp=Kp;
        KI=ki;
        ks=Ks;
    }


    void LinearQuantization(float *qValue);

    void CalcMembership(float* ms, float qv,int * index);

    void FuzzyComputation(float *DeltaK);
    

    static int ruleKp[7][7];
    static int ruleKi[7][7];

    float input;
    float setpoint;  /*设定值*/
    float kp;        /*比例系数*/
    float ki;        /*积分系数*/
    
    float ks;        /*前馈系数*/

    float lasterror; /*前一拍偏差*/
    float preerror;  /*前两拍偏差*/

    float iTerm;

    float deadband;  /*死区*/
    float output;    /*输出值*/
    float sumerror;

    float outMax;   /*输出值的上限*/
    float outMin;   /*输出值的下限*/

    float Vmax;     /*误差上限*/
    float Vmin;        /* 误差下限*/

    float maxdKp=0.2; /*Kp增量的最大限值*/
    float mindKp=-0.2; /*Kp增量的最小限值*/
    float qKp;    /*Kp增量的影响系数*/
    float maxdKi; /*Ki增量的最大限值*/
    float mindKi; /*Ki增量的最小限值*/
    float qKi;    /*Ki增量的影响系数*/


    float deltaKi; 
    float deltaKp; 

    float sampletime;




};









#endif /* PID_H_ */