/*****************************************************************************
 *
 *  $Id: main.c,v 6a6dec6fc806 2012/09/19 17:46:58 fp $
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <iostream>
#include <memory>
// #include<array>
/****************************************************************************/
#include "ecrt.h"
#include "pid_controller.h"
#include "interface/cmdpanel.h"
//#include "datalogger.h"
#include "Eigen/Dense"
#include "biped.h"

//add
#include "../interface/CheatIO.h"
#include "../include/checkjoint.h"
#include "../include/OrientationEstimator.h"
#include "../include/PositionVelocityEstimator.h"
#include "../include/utility/cppTypes.h"
#include "../include/ControlFSMData.h"
#include "../include/DesiredCommand.h"
#include "../include/FSM/FSM.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 100
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS 1

#define BIGNUM 100000000.0
PIDControl *PID_ptr_M; // 矢状面3关节，j2,j3,j4;
int sock;
struct sockaddr_in server;
char message[1000];
/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};

static ec_slave_config_t *sc1;
static ec_slave_config_state_t sc1_ana_in_state = {};

static ec_slave_config_t *sc2;
static ec_slave_config_state_t sc2_ana_in_state = {};

static ec_slave_config_t *sc3;
static ec_slave_config_state_t sc3_ana_in_state = {};

static ec_slave_config_t *sc4;
static ec_slave_config_state_t sc4_ana_in_state = {};

static ec_slave_config_t *sc5;
static ec_slave_config_state_t sc5_ana_in_state = {};

static ec_slave_config_t *sc6;
static ec_slave_config_state_t sc6_ana_in_state = {};

static ec_slave_config_t *sc7;
static ec_slave_config_state_t sc7_ana_in_state = {};


static CmdPanel *cmdptr = NULL;
// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

/****************************************************************************/

// process data
// static uint8_t *domain_pd = NULL;

// #define BusCouplerPos 0, 0

// #define TI_AM3359ICE 0x00000009, 0x26483052

// // offsets for PDO entries

static unsigned int off_bytes_0x7030 ;
static unsigned int off_bits_0x7030;

static unsigned int off_bytes_0x6000[2] = {1};
static unsigned int off_bits_0x6000[2] = {1};

static unsigned int off_bytes_0x6000_1[2] = {1};
static unsigned int off_bits_0x6000_1[2] = {1};

static unsigned int off_bytes_0x6000_2[2] = {1};
static unsigned int off_bits_0x6000_2[2] = {1};

static unsigned int off_bytes_0x6000_3[2] = {1};
static unsigned int off_bits_0x6000_3[2] = {1};

static unsigned int off_bytes_0x6000_4[2] = {1};
static unsigned int off_bits_0x6000_4[2] = {1};

static unsigned int off_bytes_0x6000_5[2] = {1};
static unsigned int off_bits_0x6000_5[2] = {1};






static unsigned int off_bytes_0x7010[6] = {1};
static unsigned int off_bits_0x7010[6] = {1};

static unsigned int off_bytes_0x7011[6] = {1};
static unsigned int off_bits_0x7011[6] = {1};

static unsigned int counter = 0;

static unsigned int blink = 0x00;
static uint8_t *domain_pd = NULL;

#define BusCouplerPos1 0, 0

#define BusCouplerPos2 256, 1

#define BusCouplerPos3 256, 0

#define BusCouplerPos4 256, 2

#define BusCouplerPos5 256, 3
#define BusCouplerPos6 256, 4
#define BusCouplerPos7 256, 5

#define TI_AM3359ICE 0x01222222, 0x00020310
#define TI_AM3359ICE1 0x00000b95, 0x00020310
#define Valve 0x00000b95, 0x00020130

#include "cstruct.h"

int fd = -1;
struct sockaddr_in saddr;
//DataLogger &logger = DataLogger::GET();

LowPassFilter filter;

float Deg[3];
float KuanDeg[2];
float rj4angle = 0, rj5angle = 0,rj3angle=0,rj2angle=0,rj1angle=0,rj0angle=0;
float LDeg[3];
float LKuanDeg[2];
float lj4angle = 0, lj5angle = 0,lj3angle=0,lj2angle=0,lj1angle=0,lj0angle=0;

float legpre2[12]={0,0,0,0,0,0,0,0,0,0,0,0};
float legpre1[12]={0,0,0,0,0,0,0,0,0,0,0,0};

unsigned long SSI[12];

void check_domain_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain_state = ds;
}

/*****************************************************************************/

/*****************************************************************************/

void check_master_state(void)
{
    // printf("check_master_state\n");
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

void check_slave1_config_states(void)
{
    // printf("check_slave_config_states\n");
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc1, &s);

    if (s.al_state != sc1_ana_in_state.al_state)
        printf("AnaIn1: State 0x%02X.\n", s.al_state);

    if (s.online != sc1_ana_in_state.online)
        printf("AnaIn1: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc1_ana_in_state.operational)
        printf("AnaIn1: %soperational.\n",
               s.operational ? "" : "Not ");

    sc1_ana_in_state = s;
}

void check_slave2_config_states(void)
{
    // printf("check_slave_config_states\n");
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc2, &s);

    if (s.al_state != sc2_ana_in_state.al_state)
        printf("AnaIn2: State 0x%02X.\n", s.al_state);

    if (s.online != sc2_ana_in_state.online)
        printf("AnaIn2: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc2_ana_in_state.operational)
        printf("AnaIn2: %soperational.\n",
               s.operational ? "" : "Not ");

    sc2_ana_in_state = s;
}

void check_slave3_config_states(void)
{
    // printf("check_slave_config_states\n");
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc3, &s);

    if (s.al_state != sc3_ana_in_state.al_state)
        printf("AnaIn3: State 0x%02X.\n", s.al_state);

    if (s.online != sc3_ana_in_state.online)
        printf("AnaIn3: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc3_ana_in_state.operational)
        printf("AnaIn3: %soperational.\n",
               s.operational ? "" : "Not ");

    sc3_ana_in_state = s;
}

void cyclic_task(Biped &bipins, float time,FSM* _FSMController)
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // check process data state (optional)
    check_domain_state();

    if (counter)
    {
        counter--;
    }
    else
    { // do this at 1 Hz

        // double LEGlength = bipins.calfLinkLength + bipins.thighLinkLength + bipins.toeLinkLength;
        // Vec6<double> RjQ;
        // Vec3<double> P;
        // float Deg[3];
        // float KuanDeg[2];
        // float rj4angle=0,rj5angle=0;
        // P.setZero();
        // P(2) = -1 * LEGlength - 0.07 * LEGlength * (cos(0.2 * M_PI * time + 1.5 * M_PI) - 1);
        // Deg[0] = 1*(14 * cos(0.2 * M_PI * time) - 14);
        // Deg[0] = 1*(14 * sin(0.2 * M_PI * time));
        rj0angle=0;
        rj1angle=0;
        rj2angle=0;
        rj3angle=0;
        rj4angle = 0;
        rj5angle = 0;
     
        lj0angle=0;
        lj1angle=0;
        lj2angle=0;
        lj3angle=0;
        lj4angle = 0;
        lj5angle = 0;

        for (int i = 0; i < 12; i++)
            SSI[i] = 0;

        uint16_t output1 = 0;
        SSI[0] = EC_READ_U32(domain_pd + off_bytes_0x6000[0]);//rj0
        SSI[1] = EC_READ_U32(domain_pd + off_bytes_0x6000[1]);
        SSI[2] = EC_READ_U32(domain_pd + off_bytes_0x6000_1[0]);
        SSI[3] = EC_READ_U32(domain_pd + off_bytes_0x6000_1[1]);
        SSI[4] = EC_READ_U32(domain_pd + off_bytes_0x6000_2[0]);
        SSI[5] = EC_READ_U32(domain_pd + off_bytes_0x6000_2[1]);
        SSI[6] = EC_READ_U32(domain_pd + off_bytes_0x6000_3[0]);//lj0
        SSI[7] = EC_READ_U32(domain_pd + off_bytes_0x6000_3[1]);
        SSI[8] = EC_READ_U32(domain_pd + off_bytes_0x6000_4[0]);
        SSI[9] = EC_READ_U32(domain_pd + off_bytes_0x6000_4[1]);
        SSI[10] = EC_READ_U32(domain_pd + off_bytes_0x6000_5[0]);
        SSI[11] = EC_READ_U32(domain_pd + off_bytes_0x6000_5[1]);
     
        // for (int i = 0; i < 12; i++)
        //    std::cout<<"编码器"<<i<< SSI[i] <<'\n';

        if (time)
        {
            // Deg[1]=15*sin(0.2*M_PI*time+1.5*M_PI);
            // Deg[0]=15*sin(M_PI*time);
            // LDeg[0]=3*sin(M_PI*time);
            // LDeg[2]=15*sin(M_PI*time);
            rj3angle=20*sin(1*M_PI*time+1.5*M_PI)+20;

            legpre2[2]=20*sin(1*M_PI*(time+0.1+bipins.sampletime)+1.5*M_PI)+20;
            legpre1[2]=20*sin(1*M_PI*(time+0.1-bipins.sampletime)+1.5*M_PI)+20;
           // LDeg[2]=8*sin(0.5*M_PI*time);
           // LDeg[2]=-20;
            // KuanDeg[0] = 2.5 * sin(M_PI * time);
            // rj4angle=-12*sin(M_PI*time);
            //    _FSMController->run();
            //    rj5angle=-10*sin(M_PI*time);
          //  _FSMController->run();
        }

        bipins.computepumpvel(legpre1,legpre2);
        bipins.pumpvelFF=bipins.pumpvelFF<<16;
        EC_WRITE_U32(domain_pd +off_bytes_0x7030, bipins.pumpvelFF);
        // Deg[2] = 0;
        // Deg[1] = 0;
        // Deg[2] =0* (18 * cos(0.2 * M_PI * time) - 18);
        // Deg[1] = -1 * (Deg[0] + Deg[2])*0;
        // P(2)*=-1;
        // bipins.ComputeIK(RjQ, P);

        // std::cout << "坐标\n"
        //           << P << '\n';
        
        float RJDES[6];
        float LJDES[6];
        RJDES[0] = rj0angle;
        RJDES[1] = rj1angle;
        RJDES[2] = rj2angle; // RJ0
        RJDES[3] = rj3angle; // RJ1
        RJDES[4] = rj4angle;          // Rj4
        RJDES[5] = rj5angle;          // Rj5

        LJDES[0] = lj1angle;
        LJDES[1] = lj2angle;
        LJDES[2] = lj3angle; // LJ3
        LJDES[4] = lj4angle;   // Lj4
        LJDES[5] = lj5angle;   // Lj5

        // unsigned long ssi[4];
        // std::vector<unsigned long> ssi;

        float *TR_data[12];
        float fSSI[12];
        for (int i = 0; i < 12; i++)
        {

            fSSI[i] = (float)SSI[i];
            TR_data[i] = &fSSI[i];
            TR_data[i] = (float *)&SSI[i];
        }

        // for (int i = 0; i < 12; i++)
        //     std::cout << "编码器" << i <<"  "<< *TR_data[i] << '\n';

        float Ldes = bipins.RJ2convert(RJDES[0]);
        float Lreal = bipins.RJ2convert(*TR_data[0]);

        float L4real, L5real, L4des, L5des;

        float gang0des, gang1des,lgang0des,lgang1des;
        float gang0real, gang1real,lgang0real,lgang1real;
       // filter.update(*TR_data[2]);
        // std::cout<<"rj0 real: "<<*TR_data[2]<<'\n';
        //  std::cout<<"rj0 real: "<<*TR_data[2]<<'\n';
        //  std::cout<<"rj0 des: "<<RJDES[2]<<'\n';
        //  std::cout<<"rj3 real: "<<*TR_data[1]<<'\n';
        // std::cout<<"rj3 des:"<<RJDES[1]<<'\n';
        //std::cout << "rj4 real:" << *TR_data[4] << '\n';
        //std::cout << "rj5 real:" << *TR_data[5] << '\n';
        bipins.RJ0RJ1convert(RJDES[0], RJDES[1], gang0des, gang1des);
        bipins.RJ0RJ1convert(*TR_data[0], *TR_data[1], gang0real, gang1real);

        // bipins.RJ0RJ1convert(*TR_data[2],0,gang0real,gang1real);
        // std::cout<<"髋部油缸0 gang0des:" <<gang0des<<'\n';
        // std::cout<<"髋部油缸0 gang0real:" <<gang0real<<'\n';
        // std::cout<<"髋部油缸1 gang1des:" <<gang1des<<'\n';
        // std::cout<<"髋部油缸1 gang1real:" <<gang1real<<'\n';

        PIDSetpointSet(&PID_ptr_M[0], gang0des);
        PIDInputSet(&PID_ptr_M[0], gang0real); // gang0
        PIDCompute(&PID_ptr_M[0]);
        FeedforwardAdd(&PID_ptr_M[0], gang0des);

      
        
        // logger.rj0des=RJDES[2];
        // logger.rj0=*TR_data[2];
        // PID_ptr_M[0].output=-4;
        // std::cout<<"PID输出"<<PID_ptr_M[0].output<<'\n';
        // logger.pidoutput[0]=PID_ptr_M[0].output;

        PIDSetpointSet(&PID_ptr_M[1], gang1des);
        PIDInputSet(&PID_ptr_M[1], gang1real); // gang1
        PIDCompute(&PID_ptr_M[1]);
        FeedforwardAdd(&PID_ptr_M[1], gang1des);

        bipins.LJ0LJ1convert(LJDES[0], LJDES[1], lgang0des, lgang1des);
        bipins.LJ0LJ1convert(*TR_data[6], *TR_data[7], lgang0real, lgang1real);

        // bipins.RJ0RJ1convert(*TR_data[2],0,gang0real,gang1real);
        // std::cout<<"髋部油缸0 gang0des:" <<lgang0des<<'\n';
        // std::cout<<"髋部油缸0 gang0real:" <<lgang0real<<'\n';
        // std::cout<<"髋部油缸1 gang1des:" <<lgang1des<<'\n';
        // std::cout<<"髋部油缸1 gang1real:" <<lgang1real<<'\n';

        PIDSetpointSet(&PID_ptr_M[6], lgang0des);
        PIDInputSet(&PID_ptr_M[6], lgang0real); // gang0
        PIDCompute(&PID_ptr_M[6]);
        FeedforwardAdd(&PID_ptr_M[6], lgang0des);

        PIDSetpointSet(&PID_ptr_M[7], lgang1des);
        PIDInputSet(&PID_ptr_M[7], lgang1real); // gang1
        PIDCompute(&PID_ptr_M[7]);
        FeedforwardAdd(&PID_ptr_M[7], lgang1des);

         // std::cout<<"LJ0PID输出"<<PID_ptr_M[6].output<<'\n';
         //   std::cout<<"LJ1PID输出"<<PID_ptr_M[7].output<<'\n';

        // logger.rj1des=RJDES[3];
        // logger.rj1=*TR_data[3];
        //   PID_ptr_M[1].output=-4;
        // std::cout<<"PID输出"<<PID_ptr_M[1].output<<'\n';


        float rj2desl = bipins.RJ2convert(RJDES[2]);
        float rj2reall = bipins.RJ2convert(*TR_data[2]);
        // std::cout<<"期望伸长量"<<rj2desl<<'\n';
        // std::cout<<"实际伸长量"<<rj2reall<<'\n';

        PIDSetpointSet(&PID_ptr_M[2], rj2desl); // RJ2
        PIDInputSet(&PID_ptr_M[2], rj2reall);
        PIDCompute(&PID_ptr_M[2]);
        FeedforwardAdd(&PID_ptr_M[2], -1 * rj2desl);
        // logger.rj2des=RJDES[0];
        //  logger.rj2=*TR_data[0];
        //  std::cout<<"PID输出"<<PID_ptr_M[2].output<<'\n';
        // logger.pidoutput[2]=PID_ptr_M[2].output;

        float lj2desl = bipins.LJ2convert(LJDES[2]);
        float lj2reall = bipins.LJ2convert(*TR_data[8]);
       //  std::cout<<"期望伸长量"<<lj2desl<<'\n';
       //  std::cout<<"实际伸长量"<<lj2reall<<'\n';

        PIDSetpointSet(&PID_ptr_M[8], lj2desl); // RJ2
        PIDInputSet(&PID_ptr_M[8], lj2reall);
        PIDCompute(&PID_ptr_M[8]);
        FeedforwardAdd(&PID_ptr_M[8],  lj2desl);
        //PID_ptr_M[8].output=0.5*lj2desl;

      //  std::cout<<"LJ2PID输出"<<PID_ptr_M[8].output<<'\n';


        float RJ3Ldes = bipins.RJ3Convert(RJDES[3]);
        float RJ3L = bipins.RJ3Convert(*TR_data[3]);
        PIDSetpointSet(&PID_ptr_M[3], RJDES[3]);
        PIDInputSet(&PID_ptr_M[3], *TR_data[3]);
        PIDCompute(&PID_ptr_M[3]);
        FeedforwardAdd(&PID_ptr_M[3], RJ3Ldes);

        std::cout<<"RJ3PID输出"<<PID_ptr_M[3].output<<'\n';
        std::cout<<"RJ3实际:"<<*TR_data[3]<<'\n';
        std::cout<<"RJ3期望"<<RJDES[3]<<'\n';

        bipins.RJ4RJ5convert(RJDES[4], RJDES[5], L4des, L5des);
        bipins.RJ4RJ5convert(*TR_data[4], *TR_data[5], L4real, L5real);

        // std::cout << "L4 real" << L4real << '\n';
        // std::cout << "L4 des" << L4des << '\n';
        // std::cout << "L5 real" << L5real << '\n';
        // std::cout << "L5 des" << L5des << '\n';

        PIDSetpointSet(&PID_ptr_M[4], L4des);
        PIDInputSet(&PID_ptr_M[4], L4real);
        PIDCompute(&PID_ptr_M[4]);
        FeedforwardAdd(&PID_ptr_M[4], L4des);

        PIDSetpointSet(&PID_ptr_M[5], L5des);
        PIDInputSet(&PID_ptr_M[5], L5real);
        PIDCompute(&PID_ptr_M[5]);
        FeedforwardAdd(&PID_ptr_M[5], L5des);


        // logger.rj3des=RJDES[1];
        // logger.rj3=*TR_data[1];
        //       PID_ptr_M[3].output=2;
        // std::cout<<"膝部缸期望伸长量:"<<RJ3Ldes<<'\n';
        //  std::cout<<"膝部缸实际伸长量"<<RJ3L<<'\n';

        //   std::cout<<"PID输出"<<PID_ptr_M[3].output<<'\n';
        // logger.pidoutput[3]=PID_ptr_M[3].output;

        // for (int i = 0; i < 12; i++)
        // {

        //     output1 = (uint16_t)((PID_ptr_M[i].output + 10) / 20 * 65536);

        //     if (i < 2)
        //     {
        //         EC_WRITE_U16(domain_pd + off_bytes_0x7010[i], output1);
        //         // printf("Current1: value=0x%x\n", EC_READ_U16(domain_pd + off_bytes_0x7010[i]));
        //     }
        //     else
        //     {

        //         EC_WRITE_U16(domain_pd + off_bytes_0x7011[i - 2], output1);
        //         // printf("Volage1: value=0x%x\n", EC_READ_U16(domain_pd + off_bytes_0x7011[i-2]));
        //     }
        // }
        // 定义要赋值给 b 的索引
    

//         PID_ptr_M[1].output=5*sin(M_PI*time+1.5*M_PI);
        // PID_ptr_M[5].output=5*sin(0.1*M_PI*time+1.5*M_PI);
        // PID_ptr_M[3].output=5*sin(M_PI*time+1.5*M_PI);
//  std::cout<<"RJ1PID输出"<<PID_ptr_M[1].output<<'\n';
        int OutIndex[6] = {0, 1, 5, 6, 7, 11};

        // 创建一个布尔数组来标记 
        bool is_out_index[12] = {false};
        for (int i = 0; i < 6; ++i)
        {
            is_out_index[OutIndex[i]] = true;
        }

        int c_idx = 0;
        int v_idx = 0;

        for (int i = 0; i < 12; ++i)
        {
            output1 = (uint16_t)((PID_ptr_M[i].output+ 10) / 20 * 65536);
            if (is_out_index[i])
            {
                EC_WRITE_U16(domain_pd + off_bytes_0x7010[c_idx++], output1);
            }
            else
            {
                EC_WRITE_U16(domain_pd + off_bytes_0x7011[v_idx++], output1);
            }
        }

        // output1 = (uint16_t)((PID_ptr_M[4].output+ 10) / 20 * 65536);
        // EC_WRITE_U16(domain_pd + off_bytes_0x7011[2], output1);

        // output1 = (uint16_t)((PID_ptr_M[5].output + 10) / 20 * 65536);
        // EC_WRITE_U16(domain_pd + off_bytes_0x7010[2], output1);
        counter = 0;

        std::cout << "-------------------\n";

        // logger.Savedata();

        char sendBuf[128];
        
        // sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f\n", RJDES[0],*TR_data[0],*TR_data[2],*TR_data[3],
        // PID_ptr_M[2].alteredKp,PID_ptr_M[2].dispKi,rj2desl,PID_ptr_M[2].FF,PID_ptr_M[2].output);

         sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", LJDES[0], *TR_data[6], LJDES[1],
                 *TR_data[7],LJDES[2], *TR_data[8],PID_ptr_M[6].output,PID_ptr_M[8].FF,lj2desl,lj2reall);

       // sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", KuanDeg[0], KuanDeg[1], Deg[0], Deg[1], rj4angle, rj5angle, LKuanDeg[0], LKuanDeg[1], LDeg[0], LDeg[1], lj4angle, lj5angle,Deg[2],LDeg[2]);

        sendto(fd, sendBuf, strlen(sendBuf) + 1, 0, (struct sockaddr *)&saddr, sizeof(saddr));
    }

    blink++;

    check_master_state();

    ecrt_domain_queue(domain);
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}

/****************************************************************************/

void signal_handler(int signum)
{
    switch (signum)
    {
    case SIGALRM:
        sig_alarms++;
        break;
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{

    fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (fd == -1)
    {
        perror("socket");
        exit(-1);
    }

    // 服务器的地址信息

    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(9999);
    inet_pton(AF_INET, "192.168.7.1", &saddr.sin_addr.s_addr);

    //control设置
    double dt = 0.001;
    Biped robot;
    CheatIO *IOptr = new CheatIO("biped");
    LegController *Legctrlptr = new LegController(robot);
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    StateEstimate stateEstimate;
    StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(state,
                                                                          Legctrlptr->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();

    
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);



    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &robot;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController =  Legctrlptr;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = IOptr;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM* _FSMController = new FSM(_controlData);

    //PID传参
    float Kp = 0.01, Ki = 0, Ks = 0;
    if (argc < 2)
        std::cout << "No Kp" << std::endl;
    else if (argc == 2)
        Kp = atof(argv[1]);
    else if (argc == 3)
    {
        Kp = atof(argv[1]);
        Ki = atof(argv[2]);
    }
    else
    {
        Kp = atof(argv[1]);
        Ki = atof(argv[2]);
        Ks = atof(argv[3]);
    }

    // Kp = std::atof(argv[1]);

    // std::unique_ptr<CmdPanel> cmd_ptr=std::make_unique<KeyBoard>();
    cmdptr = new KeyBoard();

    struct sigaction sa;
    struct itimerval tv;

    master = ecrt_request_master(0);
    if (!master)
        return -1;
    // 建立一个域，参数是masters
    domain = ecrt_master_create_domain(master);
    if (!domain)
        return -1;

        // ⽤宏定义配置master和slave 数字信号输⼊的从站/供应商ID产品代码
#if CONFIGURE_PDOS
    if (!(sc1 = ecrt_master_slave_config(
              master, BusCouplerPos1, TI_AM3359ICE)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (!(sc2 = ecrt_master_slave_config(
              master, BusCouplerPos2, TI_AM3359ICE1)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (!(sc3 = ecrt_master_slave_config(
              master, BusCouplerPos3, Valve)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (!(sc4 = ecrt_master_slave_config(
              master, BusCouplerPos4, TI_AM3359ICE1)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (!(sc5 = ecrt_master_slave_config(
              master, BusCouplerPos5, TI_AM3359ICE1)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (!(sc6 = ecrt_master_slave_config(
              master, BusCouplerPos6, TI_AM3359ICE1)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (!(sc7 = ecrt_master_slave_config(
              master, BusCouplerPos7, TI_AM3359ICE1)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    

    // 设置PDOS，利⽤sc、EC_END、对⻬信息
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc1, EC_END, slave_0_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc2, EC_END, slave_2_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc3, EC_END, slave_1_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

     printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc4, EC_END, slave_3_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

     printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc5, EC_END, slave_4_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

     printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc6, EC_END, slave_5_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

     printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc7, EC_END, slave_6_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    off_bytes_0x7030 = ecrt_slave_config_reg_pdo_entry(sc1, 0x7030, 1, domain, &off_bits_0x7030);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x7030, off_bits_0x7030);
    if (off_bytes_0x7030 < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000[0] = ecrt_slave_config_reg_pdo_entry(sc1, 0x6000, 1, domain, &off_bits_0x6000[0]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000[0], off_bits_0x6000[0]);
    if (off_bytes_0x6000[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000[1] = ecrt_slave_config_reg_pdo_entry(sc1, 0x6000, 2, domain, &off_bits_0x6000[1]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000[1], off_bits_0x6000[1]);
    if (off_bytes_0x6000[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_1[0] = ecrt_slave_config_reg_pdo_entry(sc2, 0x6000, 1, domain, &off_bits_0x6000_1[0]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_1[0], off_bits_0x6000_1[0]);
    if (off_bytes_0x6000_1[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_1[1] = ecrt_slave_config_reg_pdo_entry(sc2, 0x6000, 2, domain, &off_bits_0x6000_1[1]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_1[1], off_bits_0x6000_1[1]);
    if (off_bytes_0x6000_1[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    
    off_bytes_0x6000_2[0] = ecrt_slave_config_reg_pdo_entry(sc4, 0x6000, 1, domain, &off_bits_0x6000_2[0]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_2[0], off_bits_0x6000_2[0]);
    if (off_bytes_0x6000_2[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_2[1] = ecrt_slave_config_reg_pdo_entry(sc4, 0x6000, 2, domain, &off_bits_0x6000_2[1]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_2[1], off_bits_0x6000_2[1]);
    if (off_bytes_0x6000_2[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_3[0] = ecrt_slave_config_reg_pdo_entry(sc5, 0x6000, 1, domain, &off_bits_0x6000_3[0]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_3[0], off_bits_0x6000_3[0]);
    if (off_bytes_0x6000_3[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_3[1] = ecrt_slave_config_reg_pdo_entry(sc5, 0x6000, 2, domain, &off_bits_0x6000_3[1]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_3[1], off_bits_0x6000_3[1]);
    if (off_bytes_0x6000_3[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_4[0] = ecrt_slave_config_reg_pdo_entry(sc6, 0x6000, 1, domain, &off_bits_0x6000_4[0]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_4[0], off_bits_0x6000_4[0]);
    if (off_bytes_0x6000_4[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_4[1] = ecrt_slave_config_reg_pdo_entry(sc6, 0x6000, 2, domain, &off_bits_0x6000_4[1]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_4[1], off_bits_0x6000_4[1]);
    if (off_bytes_0x6000_4[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

       off_bytes_0x6000_5[0] = ecrt_slave_config_reg_pdo_entry(sc7, 0x6000, 1, domain, &off_bits_0x6000_5[0]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_5[0], off_bits_0x6000_5[0]);
    if (off_bytes_0x6000_5[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6000_5[1] = ecrt_slave_config_reg_pdo_entry(sc7, 0x6000, 2, domain, &off_bits_0x6000_5[1]);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000_5[1], off_bits_0x6000_5[1]);
    if (off_bytes_0x6000_5[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }



    off_bytes_0x7010[0] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 1, domain, &off_bits_0x7010[0]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7010[0], off_bits_0x7010[0]);
    if (off_bytes_0x7010[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[1] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 2, domain, &off_bits_0x7010[1]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7010[1], off_bits_0x7010[1]);
    if (off_bytes_0x7010[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[2] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 3, domain, &off_bits_0x7010[2]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7010[2], off_bits_0x7010[2]);
    if (off_bytes_0x7010[2] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[3] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 4, domain, &off_bits_0x7010[3]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7010[3], off_bits_0x7010[3]);
    if (off_bytes_0x7010[3] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[4] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 5, domain, &off_bits_0x7010[4]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7010[4], off_bits_0x7010[4]);
    if (off_bytes_0x7010[4] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[5] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 6, domain, &off_bits_0x7010[5]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7010[5], off_bits_0x7010[5]);
    if (off_bytes_0x7010[5] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7011[0] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7011, 1, domain, &off_bits_0x7011[0]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7011[0], off_bits_0x7011[0]);
    if (off_bytes_0x7011[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7011[1] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7011, 2, domain, &off_bits_0x7011[1]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7011[1], off_bits_0x7011[1]);
    if (off_bytes_0x7011[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7011[2] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7011, 3, domain, &off_bits_0x7011[2]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7011[2], off_bits_0x7011[2]);
    if (off_bytes_0x7011[2] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7011[3] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7011, 4, domain, &off_bits_0x7011[3]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7011[3], off_bits_0x7011[3]);
    if (off_bytes_0x7011[3] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7011[4] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7011, 5, domain, &off_bits_0x7011[4]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7011[4], off_bits_0x7011[4]);
    if (off_bytes_0x7011[4] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7011[5] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7011, 6, domain, &off_bits_0x7011[5]);
    printf("off_bytes_0x7011_value=0x%x %x\n", off_bytes_0x7011[5], off_bits_0x7011[5]);
    if (off_bytes_0x7011[5] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

#endif

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        printf("ecrt_master_activate fail...\n");
        return -1;
    }

    if (!(domain_pd = ecrt_domain_data(domain)))
    {
        printf("ecrt_domain_data fail...\n");
        return -1;
    }

    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0))
    {
        fprintf(stderr, "Failed to install signal handler!\n");
        return -1;
    }

    printf("Starting timer...\n");
    tv.it_interval.tv_sec = 0;
    tv.it_interval.tv_usec = 1000000 / FREQUENCY; // 10 ms  运行一次signal_handler
    tv.it_value.tv_sec = 0;
    tv.it_value.tv_usec = 2000;
    if (setitimer(ITIMER_REAL, &tv, NULL))
    {
        fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
        return 1;
    }

    const float controltime = 0.01, sampletime = 0.01;
    // pid_ptr = (PIDControl*)malloc(sizeof(PIDControl));
    // PIDInit(pid_ptr,Kp,Kd,0,0.02,-10,10,AUTOMATIC,DIRECT);
    filter.set(1 / sampletime, 50);
    PID_ptr_M = new PIDControl[12];

    /*
   PIDK: 0.8,0.2,0,-0.1
   */
    PIDInit(&PID_ptr_M[0], 0.8, 0.2, 0, -0.1, controltime, -10, 10, AUTOMATIC, REVERSE);
    // PIDInit(&PID_ptr_M[1], 0.065, 0, 0.0035, controltime, -10, 10, AUTOMATIC, DIRECT);
    // PIDInit(&PID_ptr_M[2], 0.048, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);

    PIDInit(&PID_ptr_M[1], 0.8, 0.2, 0, -0.1, controltime, -10, 10, AUTOMATIC, REVERSE);
    /*
   PIDK: 0.065,0.024,0,-0.02
   */
    PIDInit(&PID_ptr_M[2], 0.065, 0.024, 0, -0.02, controltime, -10, 10, AUTOMATIC, DIRECT);
    /*
    PIDK: 0.06,0.004,0,0.0811
    */
    PIDInit(&PID_ptr_M[3], 0.06, 0.004, 0, 0.08, controltime, -10, 10, AUTOMATIC, REVERSE);
    /*
     PIDK: 0.4,0,0,0.02
     */
    PIDInit(&PID_ptr_M[4], 0.4, 0, 0, 0.02, controltime, -10, 10, AUTOMATIC, DIRECT);
    /*
    PIDK: 1.0,0.08,0,0.1
    */
    PIDInit(&PID_ptr_M[5], 1, 0.08, 0, 0.1, controltime, -10, 10, AUTOMATIC, DIRECT);

    /*
    PIDK: 0.8,0.2,0,-0.1
    */
    PIDInit(&PID_ptr_M[6], 0.6, 0.05, 0, 0.1, controltime, -10, 10, AUTOMATIC,DIRECT);
    // PIDInit(&PID_ptr_M[1], 0.065, 0, 0.0035, controltime, -10, 10, AUTOMATIC, DIRECT);
    // PIDInit(&PID_ptr_M[2], 0.048, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);

    PIDInit(&PID_ptr_M[7], 0.6, 0.05, 0, 0.1, controltime, -10, 10, AUTOMATIC, DIRECT);
    /*
    PIDK: 0.065,0.024,0,-0.02
   */
    PIDInit(&PID_ptr_M[8], Kp, Ki, 0, Ks, controltime, -10, 10, AUTOMATIC, REVERSE);
    /*
    PIDK: 0.06,0.004,0,0.0811
    */
    PIDInit(&PID_ptr_M[9], 0.06, 0.004, 0, 0.08, controltime, -10, 10, AUTOMATIC, REVERSE);
    /*
    PIDK: 0.4,0,0,0.02
    */
    PIDInit(&PID_ptr_M[10], 0.4, 0, 0, 0.02, controltime, -10, 10, AUTOMATIC, DIRECT);
    /*
    PIDK: 1.0,0.08,0,0.1
    */
    PIDInit(&PID_ptr_M[11], 1, 0.08, 0, 0.1, controltime, -10, 10, AUTOMATIC, DIRECT);

    printf("Started.\n");

    Biped bipedinstance;

    bipedinstance.sampletime=0.01;

    float time = 0;

    while (1)
    {
        pause();
        while (sig_alarms != user_alarms)
        {
            if (cmdptr->uservalue.Starttime)
                time += sampletime;
            else
                time = 0;

            // cyclic_task(bipedinstance, time, NULL);
            cyclic_task(bipedinstance, time, _FSMController);
            user_alarms++;
        }
        // std::cout<<"Keyboard Value\n"<<cmd_ptr->uservalue.direction<<'\n';
    }

    delete[] PID_ptr_M;
    delete cmdptr;
    delete _FSMController;
    delete _controlData;
    delete desiredStateCommand;
    delete stateEstimator;
    delete state;
    delete cmd;
    delete Legctrlptr;
    delete IOptr;

    return 0;
}

/****************************************************************************/
