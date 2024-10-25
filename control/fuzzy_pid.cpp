#include "fuzzy_pid.h"



#define NB -6
#define NM -4
#define NS -2
#define ZO 0
#define PS 2
#define PM 4
#define PB 6


int fuzzypid::ruleKp[7][7] = {{PB, PB, PM, PM, PS, ZO, ZO},
					{PB, PB, PM, PS, PS, ZO, ZO},
					{PM, PM, PM, PS, ZO, NS, NS},
					{PM, PM, PS, ZO, NS, NM, NM},
					{PS, PS, ZO, NS, NS, NM, NM},
					{PS, ZO, NS, NM, NM, NM, NB},
					{ZO, ZO, NM, NM, NM, NB, NB}};


int fuzzypid::ruleKi[7][7] = {{NB, NB, NM, NM, NS, ZO, ZO},
					{NB, NB, NM, NS, NS, ZO, ZO},
					{NB, NM, NS, NS, ZO, PS, PS},
					{NM, NM, NS, ZO, PS, PM, PM},
					{NM, NS, ZO, PS, PS, PM, PB},
					{ZO, ZO, PS, PS, PM, PB, PB},
					{ZO, ZO, PS, PM, PM, PB, PB}};


float LinearRealization(float max,float min,float value)
{
    float x=(max-min)*value/6;
    return x;
}

void fuzzypid::LinearQuantization(float *qValue)
{
    float pv=input;
    float thisError;
    float deltaError;

    thisError=setpoint-pv;
    deltaError=thisError-lasterror;

    qValue[0]=6*thisError/(Vmax-Vmin);
    qValue[1]=3*deltaError/(Vmax-Vmin);

}




void fuzzypid::CalcMembership(float* ms,float qv, int * index)
{
    float 

}