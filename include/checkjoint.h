
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
#include "lip3d.h"

#define zmpSx 0.15
#define zmpSy 0.1
#define PREVIEWNUM 80


using namespace Eigen;


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
        //walking=new Gait(60, Vec2<int>(0, 30), Vec2<int>(30, 30), "Walking");
        float para=0.5;
        walkzmp=new Gait(para*60, Vec2<int>(0, para*40), Vec2<int>(para*40, para*40), "zmpWalking");
        zmppre=new Gait(para*60, Vec2<int>(0, para*40), Vec2<int>(para*40, para*40), "zmpWalking");
        int wholephase=2*para*60;
        int stancephase=para*(60+0.2*60);
        int offsetL=wholephase-0.05*60*para;
        int offsetR=para*60-0.05*60*para;
        zmpfoot=new Gait(wholephase,Vec2<int>(offsetR,offsetL),Vec2<int>(stancephase, stancephase),"ZMPfoot_schedule");
       // gaitsquat=new Gait(60*2, Vec2<int>(0, 50*2), Vec2<int>(2*10, 2*50), "Walking");

        for(int i=0;i<PREVIEWNUM;i++)
        {
            walkzmp->setIterations(iterationsBetweenMPC,iterationCounter+i);
            float thephase=walkzmp->_phase;
            lastphaselist[i]=thephase;
            Iterlist[i]=0;
            xreflist[i]=0;
            yreflist[i]=0;
        }

        dymodelx=new LIP3d(PREVIEWNUM);
        dymodely=new LIP3d(PREVIEWNUM);
        dtMPC = dt * iterationsBetweenMPC;
        _lowCmd=cmdex;
        _lowState=statex;

        DesiredPos.setZero();
        // plotsub[0]=_nc.advertise<std_msgs::Float32>( "plot1", 10);
        // plotsub[1]=_nc.advertise<std_msgs::Float32>( "plot2", 10);
        // plotsub[2]=_nc.advertise<std_msgs::Float32>( "plot3", 10);
        Footpos[0].setZero();
        Footpos[1].setZero();
        GetzmpinitAngle();

    }
    ~Checkjoint()
    {
        // _lowCmd=nullptr;
        // _lowState=nullptr;
        delete walking;
        delete walkzmp;
        delete gaitsquat;
        delete dymodelx;
        delete dymodely;
        _lowCmd = NULL;
        _lowState = NULL;
    }
    //void checkgait();

    void GetzmpinitAngle();

    void zmpLegphasecompute();

    void checkgait1();

    void squat();

    void zmpgenerate(int deltapre);

    void zmpwalk();

    void Setjointpos( Vec6<double> QDes[2]);

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
    LIP3d* dymodelx;
    LIP3d* dymodely;

    Gait* walking;
    Gait* walkzmp;
    Gait* gaitsquat;
    Gait * zmppre;
    Gait* zmpfoot;
    int iterationsBetweenMPC=4;
    int horizonLength=30;
    FootSwingTrajectory<double> footSwingTrajectories[2];
    double dt=0.01;
    double dtMPC;
    bool firstRun = true;
    Vec3<double> world_position_desired;
    int iterationCounter = 1;
    Vec3<double> pFoot[2];
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    LegController* statectrl;
    IOInterface* ioptr;
    Vec2<double> swingTimes;
    Vec3<float> Footpos[2];

    StateEstimatorContainer* _stateEstimator;
    Vec3<float> DesiredPos;//世界坐标系下期望位置
    Vec3<double> pDesFootWorld[2] ;
    Vec3<double> vDesFootWorld[2] ;
    Vec3<double> pDesFoot[2] ;
    Vec3<double> vDesFoot[2] ;

    bool firstSwing[2] = {false, true};
    bool firstSwingzmp[2]={true,false};

    bool firstSwingpre[2] = {true, true};
    bool startwalk=false;
    float footxnow[2]={0,0};
    float footxnext[2]={-0.1,0.1};
    float footxnextsquat[2]={0,0};

    float zmpxposref[7]={0,0,zmpSx,2*zmpSx,3*zmpSx,4*zmpSx,5*zmpSx};
    float zmpyposref[7]={0,zmpSy,-zmpSy,zmpSy,-zmpSy,zmpSy,0};
    const short totalstep=7;
    
    double swingTimeRemaining[2];
    float qdes=0;
    int walktimes=0;
    short up=1;
   // int previewnum=50;

    float sum_ex=0;
    float sum_ey=0;

    float zmpChangep=0.1;
 //   float * Lastphaselist;

    float lastphaselist[PREVIEWNUM];
    //int * Iterlist;
    int  Iterlist[PREVIEWNUM];
    //float * xreflist;

    float xreflist[PREVIEWNUM];
    //float * yreflist;
    float yreflist[PREVIEWNUM];

    int Iter=0;

    float refheight=1.12;

    


};

#endif
