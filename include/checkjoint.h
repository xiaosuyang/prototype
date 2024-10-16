
#ifndef CHECK_JOINT_H
#define CHECK_JOINT_H
// #include <vector>
#include <iostream>
#include <memory>
#include <cmath>
#include "LegController.h"
#include "GaitGenerator.h"
#include "../interface/IOInterface.h"
#include "../interface/LowLevelCmd.h"
#include "../interface/LowlevelState.h"
#include "FootSwingTrajectory.h"
#include "StateEstimatorContainer.h"


extern float Deg[3];
extern float KuanDeg[2];
extern float rj0angle,rj1angle,rj2angle,rj3angle,rj4angle, rj5angle;
extern float LDeg[3];
extern float LKuanDeg[2];
extern float lj0angle,lj1angle,lj2angle,lj3angle,lj4angle, lj5angle;


enum JOINT
{
    RJ0,
    RJ1,
    RJ2,
    RJ3,
    RJ4,
    RJ5,
    LJ0,
    LJ1,
    LJ2,
    LJ3,
    LJ4,
    LJ5
};
// using std::shared_ptr;

class Checkjoint
{
public:
    // Checkjoint() = default;
    Checkjoint(){};
    Checkjoint(IOInterface* io,LegController* ctrl,LowlevelCmd* cmdex,LowlevelState* statex,
    StateEstimatorContainer* esptr) : ioptr(io),statectrl(ctrl),_stateEstimator(esptr)
    {
        for(int i=0;i<2;i++) statectrl->data[i].zero();
        walking=new Gait(60, Vec2<int>(0, 30), Vec2<int>(30, 30), "Walking");
        walkpre=new Gait(60, Vec2<int>(0, 30), Vec2<int>(30, 30), "Walking");
        dtMPC = dt * iterationsBetweenMPC;
        _lowCmd=cmdex;
        _lowState=statex;
        // plotsub[0]=_nc.advertise<std_msgs::Float32>( "plot1", 10);
        // plotsub[1]=_nc.advertise<std_msgs::Float32>( "plot2", 10);
        // plotsub[2]=_nc.advertise<std_msgs::Float32>( "plot3", 10);
        Footpos[0].setZero();
        Footpos[1].setZero();

    }
    ~Checkjoint()
    {
        // _lowCmd=nullptr;
        // _lowState=nullptr;
        _lowCmd = NULL;
        _lowState = NULL;
    }
    void checkgait();

    void checkgait1();

    void staystill()
    {
        ioptr->sendRecv(_lowCmd,_lowState);
        statectrl->updateData(_lowState);
        for(int i=0;i<6;i++)
        {
            statectrl->commands[0].qDes(i)=0;
            statectrl->commands[0].qdDes(i)=0;
            statectrl->commands[1].qDes(i)=0;
            statectrl->commands[1].qdDes(i)=0;
        }

         statectrl->updatePosctrl(_lowCmd);
        
    }
   
    void checkOnejoint(unsigned short joint)
    {
        ioptr->sendRecv(_lowCmd,_lowState);
        statectrl->updateData(_lowState);
        float q,v,qmax=1.57,dq,dqdes;
        v=qmax/2;
        // if(joint<=static_cast<unsigned short>(JOINT::RJ5)) 
        if(joint<=static_cast<unsigned short>(RJ5)) 
        {
            q=statectrl->data[0].q(joint);
            dq=statectrl->data[0].qd(joint);
        }
        else
        {q=statectrl->data[1].q(joint%6);
        dq=statectrl->data[1].qd(joint%6);
        } 
        if(qdes>=0&&qdes<=1&&up>0) 
            {qdes+=v*dt;dqdes=v;}
        else if(qdes>=0&&qdes<=1&&up<0)
            {qdes-=v*dt;dqdes=-v;}
        else 
        {
            up*=-1;
            qdes+=up*0.005;
        }

        //  if(joint<=static_cast<unsigned short>(JOINT::RJ5)) 
         if(joint<=static_cast<unsigned short>(RJ5)) 
         {
            statectrl->commands[0].qDes(joint)=qdes;
            statectrl->commands[0].qdDes(joint)=dqdes;
         }
        else
        {
            statectrl->commands[1].qDes(joint%6)=qdes;
            statectrl->commands[1].qdDes(joint%6)=dqdes;
        }

        statectrl->commands[1].qDes(5)=0.3;
        statectrl->commands[1].qdDes(5)=0;
        statectrl->updatePosctrl(_lowCmd);
    }
    // void plot_publish(int i,float value)
    // {
    //     std_msgs::Float32 Val;
    //     Val.data=value;
    //     plotsub.at(i).publish(Val);
    // }

    // ros::NodeHandle _nc;
    // std::array<ros::Publisher,3>  plotsub;
    Gait* walking;
    Gait* walkpre;
    int iterationsBetweenMPC=4;
    int horizonLength=30;
    FootSwingTrajectory<double> footSwingTrajectories[2];
    double dt=0.01;
    double dtMPC;
    bool firstRun = true;
    Vec3<double> world_position_desired;
    int iterationCounter = 0;
    Vec3<double> pFoot[2];
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    LegController* statectrl;
    IOInterface* ioptr;
    Vec2<double> swingTimes;
    Vec3<float> Footpos[2];
    StateEstimatorContainer* _stateEstimator;
    Vec3<double> pDesFootWorld[2] ;
    Vec3<double> vDesFootWorld[2] ;
    Vec3<double> pDesFoot[2] ;
    Vec3<double> vDesFoot[2] ;

    bool firstSwing[2] = {true, true};

      bool firstSwingpre[2] = {true, true};
    
    double swingTimeRemaining[2];
    float qdes=0;
    short up=1;
};

#endif
