#include "fuzzy_pid.h"



#define NB -6
#define NM -4
#define NS -2
#define ZO 0
#define PS 2
#define PM 4
#define PB 6

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

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
    if ((qv >= -NB) && (qv < -NM))
	{
		index[0] = 0;
		index[1] = 1;
		ms[0] = -0.5 * qv - 2.0; // y=-0.5x-2.0
		ms[1] = 0.5 * qv + 3.0;	 // y=0.5x+3.0
	}
	else if ((qv >= -NM) && (qv < -NS))
	{
		index[0] = 1;
		index[1] = 2;
		ms[0] = -0.5 * qv - 1.0; // y=-0.5x-1.0
		ms[1] = 0.5 * qv + 2.0;	 // y=0.5x+2.0
	}
	else if ((qv >= -NS) && (qv < ZO))
	{
		index[0] = 2;
		index[1] = 3;
		ms[0] = -0.5 * qv;		// y=-0.5x
		ms[1] = 0.5 * qv + 1.0; // y=0.5x+1.0
	}
	else if ((qv >= ZO) && (qv < PS))
	{
		index[0] = 3;
		index[1] = 4;
		ms[0] = -0.5 * qv + 1.0; // y=-0.5x+1.0
		ms[1] = 0.5 * qv;		 // y=0.5x
	}
	else if ((qv >= PS) && (qv < PM))
	{
		index[0] = 4;
		index[1] = 5;
		ms[0] = -0.5 * qv + 2.0; // y=-0.5x+2.0
		ms[1] = 0.5 * qv - 1.0;	 // y=0.5x-1.0
	}
	else if ((qv >= PM) && (qv <= PB))
	{
		index[0] = 5;
		index[1] = 6;
		ms[0] = -0.5 * qv + 3.0; // y=-0.5x+3.0
		ms[1] = 0.5 * qv - 2.0;	 // y=0.5x-2.0
	}



}


void fuzzypid::FuzzyComputation(float *DeltaK)
{
	float qValue[2] = {0, 0}; 
	int indexE[2] = {0, 0};	  
	float msE[2] = {0, 0};	  
	int indexEC[2] = {0, 0};  
	float msEC[2] = {0, 0};	 
	float qValueK[2];

	LinearQuantization(qValue);

	CalcMembership(msE, qValue[0], indexE);
	CalcMembership(msEC, qValue[1], indexEC);

	qValueK[0] = msE[0] * (msEC[0] * ruleKp[indexE[0]][indexEC[0]] + msEC[1] * ruleKp[indexE[0]][indexEC[1]]) 
           + msE[1] * (msEC[0] * ruleKp[indexE[1]][indexEC[0]] + msEC[1] * ruleKp[indexE[1]][indexEC[1]]);

    qValueK[1] = msE[0] * (msEC[0] * ruleKi[indexE[0]][indexEC[0]] + msEC[1] * ruleKi[indexE[0]][indexEC[1]]) 
           + msE[1] * (msEC[0] * ruleKi[indexE[1]][indexEC[0]] + msEC[1] * ruleKi[indexE[1]][indexEC[1]]);

	deltaKp = LinearRealization(maxdKp, mindKp, qValueK[0]);
	deltaKi = LinearRealization(maxdKi, mindKi, qValueK[1]);

	DeltaK[0]=alteredKp+deltaKp;
	DeltaK[1]=alteredKi+deltaKi;

}


void fuzzypid::PIDturningset()
{
	alteredKp = kp;
    alteredKi = ki * sampletime;

    
    // Apply reverse direction to the altered values if necessary
    if(controllerDirection == F_REVERSE)
    {
        alteredKp = -(alteredKp);
        alteredKi = -(alteredKi);
    }

}

void fuzzypid::PIDcompute()
{
	float pv=input;
    float thisError;
    float deltaError;

    thisError=setpoint-pv;
	
	error=thisError;

    deltaerror=thisError-lasterror;

	desiredDelta=setpoint-lastSetpoint;

	float deltaK[2]= {0};
	
	FuzzyComputation(deltaK);

	iTerm+=deltaK[1]*error;
	iTerm=CONSTRAIN(iTerm,0.5*outMin,0.5*outMax);

	output=deltaK[0]*error+iTerm;

	output=CONSTRAIN(output,outMin,outMax);



	lastSetpoint=setpoint;

	lasterror=thisError;


	
}




void fuzzypid::FeedforwardAdd(bool flag)
{
	FF=ks*(desiredDelta)/sampletime;

	if(flag)
	{
		output=CONSTRAIN(output+FF,outMin,outMax);
	}

}