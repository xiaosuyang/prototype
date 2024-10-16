//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C -
// Platform Independent
// 
// Revision: 1.1
// 
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
// 
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// 
// Revisions can be found here:
// https://github.com/tcleg
// 
// Modified by: Trent Cleghorn , <trentoncleghorn@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
// 
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// this program.  If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************************

//*********************************************************************************
// Headers
//*********************************************************************************
#include "pid_controller.h"
#include <stdio.h>
//#include <iostream>
//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
float lastinput=0;
//*********************************************************************************
// Functions
//*********************************************************************************
void PIDInit(PIDControl *pid, float kp, float ki, float kd,float KS, 
             float sampleTimeSeconds, float minOutput, float maxOutput, 
             PIDMode mode, PIDDirection controllerDirection)     	
{
    pid->controllerDirection = controllerDirection;
    pid->mode = mode;
    pid->iTerm = 0.0f;
    pid->input = 0.0f;
    pid->lastInput = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    pid->Ks=KS;
    if(sampleTimeSeconds > 0.0f)
    {
        pid->sampleTime = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        pid->sampleTime = 1.0f;
    }
    
    PIDOutputLimitsSet(pid, minOutput, maxOutput);
    PIDTuningsSet(pid, kp, ki, kd);
}
        
bool
PIDCompute(PIDControl *pid) 
{
    float error;

    if(pid->mode == MANUAL)
    {
        return false;
    }
    
    // The classic PID error term
    error = (pid->setpoint) - (pid->input);
    
    // Compute the integral term separately ahead of time
    pid->iTerm += (pid->alteredKi) * error;
    
    // Constrain the integrator to make sure it does not exceed output bounds
    pid->iTerm = CONSTRAIN( (pid->iTerm), 0.5*(pid->outMin),0.5* (pid->outMax) );
    
    // Take the "derivative on measurement" instead of "derivative on error"
    pid->dInput = (pid->input) - (pid->lastInput);
    
    // Run all the terms together to get the overall output
    pid->output = (pid->alteredKp) * error + (pid->iTerm) - (pid->alteredKd) * pid->dInput;
    
    // Bound the output
    pid->output = CONSTRAIN( (pid->output), (pid->outMin), (pid->outMax) );
    
    // Make the current input the former input
    pid->lastInput = pid->input;
    
    return true;
}
     
void 
PIDModeSet(PIDControl *pid, PIDMode mode)                                                                                                                                       
{
    // If the mode changed from MANUAL to AUTOMATIC
    if(pid->mode != mode && mode == AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        pid->iTerm = pid->output;
        pid->lastInput = pid->input;
        
        // Constrain the integrator to make sure it does not exceed output bounds
        pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );
    }
    
    pid->mode = mode;
}

void 
PIDOutputLimitsSet(PIDControl *pid, float min, float max) 							  							  
{
    // Check if the parameters are valid
    if(min >= max)
    {
        return;
    }
    
    // Save the parameters
    pid->outMin = min;
    pid->outMax = max;
    
    // If in automatic, apply the new constraints
    if(pid->mode == AUTOMATIC)
    {
        pid->output = CONSTRAIN(pid->output, min, max);
        pid->iTerm  = CONSTRAIN(pid->iTerm,  min, max);
    }
}

void 
PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd)         	                                         
{
    // Check if the parameters are valid
    if(kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }
    
    // Save the parameters for displaying purposes
    pid->dispKp = kp;
    pid->dispKi = ki;
    pid->dispKd = kd;
    
    // Alter the parameters for PID
    pid->alteredKp = kp;
    pid->alteredKi = ki * pid->sampleTime;
    pid->alteredKd = kd / pid->sampleTime;
    
    // Apply reverse direction to the altered values if necessary
    if(pid->controllerDirection == REVERSE)
    {
        pid->alteredKp = -(pid->alteredKp);
        pid->alteredKi = -(pid->alteredKi);
        pid->alteredKd = -(pid->alteredKd);
    }
}

void 
PIDTuningKpSet(PIDControl *pid, float kp)
{
    PIDTuningsSet(pid, kp, pid->dispKi, pid->dispKd);
}

void 
PIDTuningKiSet(PIDControl *pid, float ki)
{
    PIDTuningsSet(pid, pid->dispKp, ki, pid->dispKd);
}

void 
PIDTuningKdSet(PIDControl *pid, float kd)
{
    PIDTuningsSet(pid, pid->dispKp, pid->dispKi, kd);
}

void 
PIDControllerDirectionSet(PIDControl *pid, PIDDirection controllerDirection)	  									  									  									  
{
    // If in automatic mode and the controller's sense of direction is reversed
    if(pid->mode == AUTOMATIC && controllerDirection == REVERSE)
    {
        // Reverse sense of direction of PID gain constants
        pid->alteredKp = -(pid->alteredKp);
        pid->alteredKi = -(pid->alteredKi);
        pid->alteredKd = -(pid->alteredKd);
    }
    
    pid->controllerDirection = controllerDirection;
}

void 
PIDSampleTimeSet(PIDControl *pid, float sampleTimeSeconds)                                                       									  									  									   
{
    float ratio;

    if(sampleTimeSeconds > 0.0f)
    {
        // Find the ratio of change and apply to the altered values
        ratio = sampleTimeSeconds / pid->sampleTime;
        pid->alteredKi *= ratio;
        pid->alteredKd /= ratio;
        
        // Save the new sampling time
        pid->sampleTime = sampleTimeSeconds;
    }
}

bool FeedforwardAdd(PIDControl *pid,float input,bool flag)
{
 
  pid->FF=1*pid->Ks*(input-pid->FFlastinput)/pid->sampleTime;
  //std::cout<<"PIDFF:\n";
  //std::cout<<pid->FF<<'\n';
  pid->FFlastinput=input;
   //printf("前馈值%f\n",FF);
   if(flag)
   {
      pid->output=CONSTRAIN(pid->output+pid->FF,(pid->outMin), (pid->outMax));
   }


}
/*
A1:无杆腔
A2：有杆腔；

*/

bool FeedforwardAdd_P(PIDControl *pid,float A1,float A2,float ps,float p0,float p1, float p2,float u1,float In,float Gain)
{
    constexpr float Qn=7e-3/60;
    constexpr float Pn=21e6;

    float Kv=Qn/(In*sqrt(Pn));

    float Id1=0,Id2=0;

    if(u1>=0)
    {
        if(ps>p1)
        {
            Id1=u1*A1/(Kv*std::sqrt(ps-p1));
        }

        if(p2>p0)
        {
         //   Id2=u1*A2/(Kv*std::sqrt(p2-p0));
        }

    }
    else
    {
        if(p1>p0)
        {
         //   Id1=u1*A1/(Kv*std::sqrt(p1-p0));
        }
        if(ps>p2)
        {
            Id2=u1*A2/(Kv*std::sqrt(ps-p2));
        }

    }

    float Id=Gain*(Id1+Id2);

    pid->FFP=Id;
    pid->output=CONSTRAIN(pid->output+pid->FFP,(pid->outMin), (pid->outMax));

}

void 
PIDSetpointSet(PIDControl *pid, float setpoint) { pid->setpoint = setpoint; }

// 
// PID Input Set
// Description:
//      Should be called before calling PIDCompute so the PID controller will
//      have an updated input value to work with.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      input - The value the controller will work with.
// Returns:
//      Nothing.
// 
void 
PIDInputSet(PIDControl *pid, float input) { pid->input = input; }

// 
// PID Output Get
// Description:
//      Typically, this function is called after PIDCompute in order to
//      retrieve the output of the controller.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The output of the specific PID controller.
// 
 float 
PIDOutputGet(PIDControl *pid) { return pid->output; }

// 
// PID Proportional Gain Constant Get111

// Description:
//      Returns the proportional gain constant value the particular
//      controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The proportional gain constant.
// 
 float 
PIDKpGet(PIDControl *pid) { return pid->dispKp; }						  

// 
// PID Integral Gain Constant Get
// Description:
//      Returns the integral gain constant value the particular
//      controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The integral gain constant.
// 
float 
PIDKiGet(PIDControl *pid) { return pid->dispKi; }						  

// 
// PID Derivative Gain Constant Get
// Description:
//      Returns the derivative gain constant value the particular
//      controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The derivative gain constant.
// 
 float 
PIDKdGet(PIDControl *pid) { return pid->dispKd; }						  

// 
// PID Mode Get
// Description:
//      Returns the mode the particular controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      MANUAL or AUTOMATIC depending on what the user set the 
//      controller to.
// 
 PIDMode 
PIDModeGet(PIDControl *pid) { return pid->mode; }						  


 PIDDirection 
PIDDirectionGet(PIDControl *pid) { return pid->controllerDirection; }		
