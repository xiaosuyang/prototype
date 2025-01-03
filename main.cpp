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
#include "fuzzy_pid.h"
// #include "datalogger.h"
#include "Eigen/Dense"
#include "biped.h"
#include "math/MathUtilities.h"
// add
#include "../interface/CheatIO.h"
#include "../include/checkjoint.h"
#include "../include/OrientationEstimator.h"
#include "../include/PositionVelocityEstimator.h"
#include "../include/utility/cppTypes.h"
#include "../include/ControlFSMData.h"
#include "../include/DesiredCommand.h"
#include "../include/FSM/FSM.h"
#include "../include/N100WP.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS 1

#define BIGNUM 100000000.0
PIDControl *PID_ptr_M; //
fuzzypid *FPID_ptr;
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

int fd1;
struct termios options1;

// u8 Fd_data[64];
// u8 Fd_rsimu[64];
// u8 Fd_rsahrs[56];
// int rs_imutype = 0;
// int rs_ahrstype = 0;

// IMUData_Packet_t IMUData_Packet;
// AHRSData_Packet_t AHRSData_Packet;
/****************************************************************************/

// process data
// static uint8_t *domain_pd = NULL;

// #define BusCouplerPos 0, 0

// #define TI_AM3359ICE 0x00000009, 0x26483052

// // offsets for PDO entries

static unsigned int off_bytes_0x7030;
static unsigned int off_bits_0x7030;

static unsigned int off_bytes_0x7030_1;
static unsigned int off_bits_0x7030_1;

static unsigned int off_bytes_0x6000[2] = {1};
static unsigned int off_bits_0x6000[2] = {1};

static unsigned int off_bytes_0x6000_1[2] = {1};
static unsigned int off_bits_0x6000_1[2] = {1};

static unsigned int off_bytes_0x6010[8] = {1};
static unsigned int off_bits_0x6010[8] = {1};

static unsigned int off_bytes_0x6010_1[2] = {1};
static unsigned int off_bits_0x6010_1[2] = {1};

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
// DataLogger &logger = DataLogger::GET();

// LowPassFilter filter;
float ifswing = 0;
float Deg[3];
float KuanDeg[2];
float rj4angle = 0, rj5angle = 0, rj3angle = 0, rj2angle = 0, rj1angle = 0, rj0angle = 0;
float LDeg[3];
float LKuanDeg[2];
float lj4angle = 0, lj5angle = 0, lj3angle = 0, lj2angle = 0, lj1angle = 0, lj0angle = 0;
float Ps = 0;
float P0 = 0;
float swingtime = 0;
float swingtime1 = 0;

float zmpscope[4]={0,0,0,0};
float cphasescope[2]={0,0};
float walkphase;
float dyx,dyy;
float uxx,uyy;
float rlegswingphase,rlegcontactphase;

bool firstrun = true;
/*
legpre2:rad
*/
float legpre2[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/*
legpre2l:mm
*/
float legpre2L[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float legpre1_5[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float *TR_data[12];

/*
legpre1:rad
*/
float legpre1[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/*
legpre1L:油缸长度,mm
*/
float legpre1L[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

unsigned long SSI[12];
float PressureA[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float PressureB[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void copy_arr_part(u8 src[], u8 dest[], int start, int end)
{
    memcpy(dest, src + start, (end - start) * sizeof(u8));
}




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

void cyclic_task(Biped &bipins, float time, FSM *_FSMController)
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
  
        rj0angle = 0;
        rj1angle = 0;
        rj2angle = 0;
        rj3angle = 0;
        rj4angle = 0;
        rj5angle = 0;

        lj0angle = 0;
        lj1angle = 0;
        lj2angle = 0;
        lj3angle = 0;
        lj4angle = 0;
        lj5angle = 0;

        int press = 0;

        for (int i = 0; i < 12; i++)
            SSI[i] = 0;

        uint16_t output1 = 0;
        SSI[0] = EC_READ_U32(domain_pd + off_bytes_0x6000[0]); // rj0
        SSI[1] = EC_READ_U32(domain_pd + off_bytes_0x6000[1]);
        SSI[2] = EC_READ_U32(domain_pd + off_bytes_0x6000_1[0]);
        SSI[3] = EC_READ_U32(domain_pd + off_bytes_0x6000_1[1]);
        SSI[4] = EC_READ_U32(domain_pd + off_bytes_0x6000_2[0]);
        SSI[5] = EC_READ_U32(domain_pd + off_bytes_0x6000_2[1]);
        SSI[6] = EC_READ_U32(domain_pd + off_bytes_0x6000_3[0]); // lj0
        SSI[7] = EC_READ_U32(domain_pd + off_bytes_0x6000_3[1]);
        SSI[8] = EC_READ_U32(domain_pd + off_bytes_0x6000_4[0]);
        SSI[9] = EC_READ_U32(domain_pd + off_bytes_0x6000_4[1]);
        SSI[10] = EC_READ_U32(domain_pd + off_bytes_0x6000_5[0]);
        SSI[11] = EC_READ_U32(domain_pd + off_bytes_0x6000_5[1]);

        for (int i = 0; i < 12; i++)
        {
            TR_data[i] = (float *)&SSI[i];
        }

        PressureA[2] = (float)(EC_READ_U16(domain_pd + off_bytes_0x6010[0])) / 65535 * 10 * 3;
        PressureB[2] = (float)(EC_READ_U16(domain_pd + off_bytes_0x6010[1])) / 65535 * 10 * 3;

        PressureA[8] = (float)(EC_READ_U16(domain_pd + off_bytes_0x6010[2])) / 65535 * 10 * 3;
        PressureB[8] = (float)(EC_READ_U16(domain_pd + off_bytes_0x6010[3])) / 65535 * 10 * 3;

        Ps = (float)(EC_READ_U16(domain_pd + off_bytes_0x6010_1[0]));
        P0 = (float)(EC_READ_U16(domain_pd + off_bytes_0x6010_1[1]));

        Ps = Ps / 100;

        // cout << "A,B口压力： " << PressureA[2] << '\n';
        // cout << PressureB[2] << '\n';

        // cout << "LA,B口压力： " << PressureA[8] << '\n';
        // cout << PressureB[8] << '\n';

        cout << "PS压力:\n"
             << Ps << '\n';

        // for (int i = 0; i < 12; i++)
        //    std::cout<<"编码器"<<i<< SSI[i] <<'\n';zz
        //   bool firstrun=true;

        if (time)
        {
            // Vec6<double> Qt;
            // Vec3<double> pt(0,0,-0.897);

            // bipins.ComputeIK(Qt,pt);

            // cout<<"关节角度"<<Qt<<'\n';
            // IKinbodyframe(statectrl->_biped, QDes[foot], &Pdes[foot], foot);
            if (bipins.AngleInit(rj1angle, rj2angle, rj3angle, rj4angle, rj5angle, lj1angle, lj2angle, lj3angle, lj4angle, lj5angle))
            {
                if (cmdptr->uservalue.Settime == true)
                {
                    _FSMController->run();
                }
            }

            // legpre1[0]=rj0angle ;
            // legpre1[1]=rj1angle ;
            // legpre1[2]=rj2angle ;
            // legpre1[3]=rj3angle ;
            // legpre1[4]=rj4angle ;
            // legpre1[5]=rj5angle ;
            /** 右腿流量前馈调试
            rj3angle=20*sin(1*M_PI*time+1.5*M_PI)+20;
            legpre2[3]=20*sin(1*M_PI*(time+0.5+bipins.sampletime)+1.5*M_PI)+20;
            legpre1[3]=20*sin(1*M_PI*(time+0.5-bipins.sampletime)+1.5*M_PI)+20;
            press=0;
            rj3angle=0;
            legpre1[3]=0;
            legpre2[3]=0;
            rj2angle=15*sin(1*M_PI*time);
            legpre2[2]=15*sin(1*M_PI*(time+0.1+bipins.sampletime));
            legpre1[2]=15*sin(1*M_PI*(time+0.1-bipins.sampletime));
            rj0angle=0*15*sin(0.5*M_PI*time);
            rj1angle=0*15*sin(0.5*M_PI*time);
            */

            //     rj1angle=4*sin(0.5*M_PI*time);
            //    legpre1[1]=4*sin(0.5*M_PI*(time  -bipins.sampletime));
            //    legpre2[1]=4*sin(0.5*M_PI*(time+bipins.sampletime));

            //    lj1angle=4*sin(0.5*M_PI*time);
            //    legpre1[7]=4*sin(0.5*M_PI*(time  -bipins.sampletime));
            //    legpre2[7]=4*sin(0.5*M_PI*(time+bipins.sampletime));

            //   rj3angle=20*sin(1*M_PI*time+1.5*M_PI)+20;
            // legpre2[3]=20*sin(1*M_PI*(time+0.5+bipins.sampletime)+1.5*M_PI)+20;
            // legpre1[3]=20*sin(1*M_PI*(time+0.5-bipins.sampletime)+1.5*M_PI)+20;

            //    lj3angle=20*sin(1*M_PI*time+1.5*M_PI)+20;
            // legpre2[9]=20*sin(1*M_PI*(time+0.5+bipins.sampletime)+1.5*M_PI)+20;
            // legpre1[9]=20*sin(1*M_PI*(time+0.5-bipins.sampletime)+1.5*M_PI)+20;

            //    rj2angle=10*sin(0.5*M_PI*time);
            //    legpre1[2]=10*sin(0.5*M_PI*(time  -bipins.sampletime));
            //    legpre2[2]=10*sin(0.5*M_PI*(time+bipins.sampletime));   

            //    lj2angle=10*sin(0.5*M_PI*time);
            //    legpre1[8]=10*sin(0.5*M_PI*(time  -bipins.sampletime));
            //    legpre2[8]=10*sin(0.5*M_PI*(time+bipins.sampletime));

            //    legpre1L[2]=bipins.RJ2convert(legpre1[2])*1e-3;
            //    legpre2L[2]=bipins.RJ2convert(legpre2[2])*1e-3;

            //     rj4angle=16.5*cos(0.5*M_PI*time)-16.5;
            //    legpre1[4]=16.5*cos(0.5*M_PI*(time-bipins.sampletime))-16.5;

            //     lj4angle=16.5*cos(0.5*M_PI*time)-16.5;
            //    legpre1[10]=16.5*cos(0.5*M_PI*(time-bipins.sampletime))-16.5;

            // legpre2[4]=16.5*cos(0.5*M_PI*(time+bipins.sampletime))-16.5;

            //    rj3angle=-1*(rj2angle+rj4angle);
            //    legpre1[3]=-1*(legpre1[2]+legpre1[4]);
            //    legpre2[3]=-1*(legpre2[2]+legpre2[4]);

            //    rj2angle=15*sin(0.5*M_PI*time);
            //    legpre1[2]=15*sin(0.5*M_PI*(time-bipins.sampletime));
            //    legpre2[2]=15*sin(0.5*M_PI*(time+bipins.sampletime));
            // lj2angle=10;
            // lj3angle=5;
            // lj4angle=10;
            // lj5angle=5;

            //    lj2angle=-15*sin(0.5*M_PI*time);
            //    legpre1[8]=-15*sin(0.5*M_PI*(time-bipins.sampletime));
            //    legpre1[8]=-15*sin(0.5*M_PI*(time+bipins.sampletime));

            //     float lj2vec=-1*cos(0.5*M_PI*time);
            //     float rj2vec=cos(0.5*M_PI*time);

            //     if(firstrun)
            //     {
            //         rj3angle=0;
            //         lj3angle=0;
            //         if(lj2angle<=-29)
            //         {
            //             firstrun=false;
            //         }
            //         swingtime1=time;
            //         swingtime=time;
            //     }
            //     else
            //     {
            //         if(lj2vec<0)
            //         {
            //             float time1;
            //             time1=time-swingtime;
            //             lj3angle=15*sin(M_PI*(time1)+1.5*M_PI)+15;
            //             legpre1[9]=15*sin(M_PI*(time1-bipins.sampletime)+1.5*M_PI)+15;
            //             legpre2[9]=15*sin(M_PI*(time1+bipins.sampletime)+1.5*M_PI)+15;

            //         }
            //         else
            //         {
            //             swingtime=time;
            //         }

            //         if(rj2vec<0)
            //         {
            //             float time1;
            //             time1=time-swingtime1;
            //             rj3angle=15*sin(M_PI*(time1)+1.5*M_PI)+15;
            //             legpre1[3]=15*sin(M_PI*(time1-bipins.sampletime)+1.5*M_PI)+15;
            //             legpre2[3]=15*sin(M_PI*(time1+bipins.sampletime)+1.5*M_PI)+15;
            //         }
            //         else
            //         {
            //             swingtime1=time;
            //         }

            //   }

            //       cout<<"LJDES[3]:"<<lj3angle<<'\n';

            //    rj0angle=0;

            //    bipins.RJ0RJ1convert(legpre1[0],legpre1[1],legpre1L[0],legpre1L[1]);
            //    bipins.RJ0RJ1convert(legpre2[0],legpre2[1],legpre2L[0],legpre2L[1]);

            //     rj3angle=20*sin(0.5*M_PI*time+1.5*M_PI)+20;
            //    legpre1[3]=20*sin(0.5*M_PI*(time+0.1-bipins.sampletime)+1.5*M_PI)+20;
            //    legpre2[3]=20*sin(0.5*M_PI*(time+0.1+bipins.sampletime)+1.5*M_PI)+20;

            //    rj5angle=8*sin(0.5*M_PI*time);
            //    legpre1[5]=8*sin(0.5*M_PI*(time-bipins.sampletime));

            //     lj5angle=8*sin(0.5*M_PI*time);
            //    legpre1[11]=8*sin(0.5*M_PI*(time-bipins.sampletime));
            //    legpre1L[3]=bipins.RJ2convert(legpre1[2])*1e-3;
            //    legpre2L[3]=bipins.RJ2convert(legpre2[2])*1e-3;

            //    rj4angle=18*cos(0.2*M_PI*time)-18;
            //     legpre1[4]=18*cos(0.2*M_PI*(time+0.5+bipins.sampletime))-18;
            //     legpre2[4]=18*cos(0.2*M_PI*(time+0.5-bipins.sampletime))-18;
            //    rj3angle=-1*(rj2angle+rj4angle);
            //    legpre1[3]=-1*(legpre1[4]+legpre1[2]);
            //    legpre2[3]=-1*(legpre2[4]+legpre2[2]);

            //   rj3angle=20*sin(0.5*M_PI*time+1.5*M_PI)+20;
            // legpre2[3]=20*sin(0.5*M_PI*(time+0.5+bipins.sampletime)+1.5*M_PI)+20;
            // legpre1[3]=20*sin(0.5*M_PI*(time+0.5-bipins.sampletime)+1.5*M_PI)+20;
            // rj2angle=15*sin(0.5*M_PI*time);
            // legpre2[2]=15*sin(0.5*M_PI*(time+0.1+bipins.sampletime));
            // legpre1[2]=15*sin(0.5*M_PI*(time+0.1-bipins.sampletime));

            // LDeg[2]=8*sin(0.5*M_PI*time);
            // LDeg[2]=-20;
            // KuanDeg[0] = 2.5 * sin(M_PI * time);
            // rj4angle=-12*sin(M_PI*time);
            //    _FSMController->run();
            //    rj5angle=-10*sin(M_PI*time);
        }

        bipins.computepumpvel(legpre1);

        // bipins.computepumpvel(legpre1,legpre2,legpre1_5);
        // cout << "泵前馈:\n"
        //      << bipins.pumpvelFF << '\n';
        bipins.pumpvelFF = bipins.pumpvelFF << 16;
        EC_WRITE_U32(domain_pd + off_bytes_0x7030_1, bipins.pumpvelFF);

        float RJDES[6];
        float LJDES[6];
        RJDES[0] = rj0angle;
        RJDES[1] = rj1angle;
        RJDES[2] = rj2angle; // RJ0
        RJDES[3] = rj3angle; // RJ1
        RJDES[4] = rj4angle; // Rj4
        RJDES[5] = rj5angle; // Rj5

        LJDES[0] = lj0angle;
        LJDES[1] = lj1angle;
        LJDES[2] = lj2angle; // LJ3
        LJDES[3] = lj3angle;
        LJDES[4] = lj4angle; // Lj4
        LJDES[5] = lj5angle; // Lj5

        // unsigned long ssi[4];
        // std::vector<unsigned long> ssi;

        // for (int i = 0; i < 12; i++)
        //     std::cout << "编码器" << i <<"  "<< *TR_data[i] << '\n';

        // float Ldes = bipins.RJ2convert(RJDES[0]);
        // float Lreal = bipins.RJ2convert(*TR_data[0]);

        float L4real, L5real, L4des, L5des;
        float RL4real, RL5real, RL4des, RL5des;
        float LL4real, LL5real, LL4des, LL5des;

        float gang0des, gang1des, lgang0des, lgang1des;
        float gang0real, gang1real, lgang0real, lgang1real;

        bipins.RJ0RJ1convert(RJDES[0], RJDES[1], gang0des, gang1des);
        bipins.RJ0RJ1convert(*TR_data[0], *TR_data[1], gang0real, gang1real);

        PIDSetpointSet(&PID_ptr_M[0], gang0des);
        PIDInputSet(&PID_ptr_M[0], gang0real); // gang0
        PIDCompute(&PID_ptr_M[0]);
        FeedforwardAdd(&PID_ptr_M[0], gang0des, true);

        PIDSetpointSet(&PID_ptr_M[1], gang1des);
        PIDInputSet(&PID_ptr_M[1], gang1real); // gang1
        PIDCompute(&PID_ptr_M[1]);
        FeedforwardAdd(&PID_ptr_M[1], gang1des, true);

        bipins.LJ0LJ1convert(LJDES[0], LJDES[1], lgang0des, lgang1des);
        bipins.LJ0LJ1convert(*TR_data[6], *TR_data[7], lgang0real, lgang1real);

        PIDSetpointSet(&PID_ptr_M[6], lgang0des);
        PIDInputSet(&PID_ptr_M[6], lgang0real); // gang0
        PIDCompute(&PID_ptr_M[6]);
        FeedforwardAdd(&PID_ptr_M[6], lgang0des, true);

        PIDSetpointSet(&PID_ptr_M[7], lgang1des);
        PIDInputSet(&PID_ptr_M[7], lgang1real); // gang1
        PIDCompute(&PID_ptr_M[7]);
        FeedforwardAdd(&PID_ptr_M[7], lgang1des, true);

        float rj2desl = bipins.RJ2convert(RJDES[2]);
        float rj2reall = bipins.RJ2convert(*TR_data[2]);

        PIDSetpointSet(&PID_ptr_M[2], rj2desl); // RJ2
        PIDInputSet(&PID_ptr_M[2], rj2reall);
        PIDCompute(&PID_ptr_M[2]);
        FeedforwardAdd(&PID_ptr_M[2], rj2desl, false);

        float u = numderivative(PID_ptr_M[2].setpoint * 1e-3, PID_ptr_M[2].lastsetpoint * 1e-3, bipins.sampletime);

        float ps = Ps;
        float p0 = 0;
        FeedforwardAdd_P(&PID_ptr_M[2], bipins.J2gangW, bipins.J2gangY, ps * 1e6, p0, PressureA[2] * 1e6, PressureB[2] * 1e6, u, 10, 1);

        //  cout<<"RJ2 FFp"<<PID_ptr_M[2].FFP<<'\n';
        // cout<<"RJ2_FF"<<PID_ptr_M[2].FF<<'\n';
        // FPID_ptr[2].setpointset(rj2desl);
        // FPID_ptr[2].inputset(rj2reall);
        // FPID_ptr[2].PIDcompute();
        // FPID_ptr[2].FeedforwardAdd(true);

        //  PID_ptr_M[2].output=FPID_ptr[2].output;

        float lj2desl = bipins.LJ2convert(LJDES[2]);
        float lj2reall = bipins.LJ2convert(*TR_data[8]);

        PIDSetpointSet(&PID_ptr_M[8], lj2desl); // RJ2
        PIDInputSet(&PID_ptr_M[8], lj2reall);
        PIDCompute(&PID_ptr_M[8]);
        FeedforwardAdd(&PID_ptr_M[8], lj2desl, false);
        FeedforwardAdd_P(&PID_ptr_M[8], bipins.J2gangW, bipins.J2gangY, ps * 1e6, p0, PressureA[8] * 1e6, PressureB[8] * 1e6, u, 10, -1);

        float RJ3Ldes = bipins.RJ3Convert(RJDES[3]);
        float RJ3L = bipins.RJ3Convert(*TR_data[3]);
        PIDSetpointSet(&PID_ptr_M[3], RJ3Ldes);
        PIDInputSet(&PID_ptr_M[3], RJ3L);
        PIDCompute(&PID_ptr_M[3]);
        FeedforwardAdd(&PID_ptr_M[3], RJ3Ldes, true);

        float LJ3des = bipins.LJ3Convert(LJDES[3]);
        float LJ3L = bipins.LJ3Convert(*TR_data[9]);
        PIDSetpointSet(&PID_ptr_M[9], LJ3des);
        PIDInputSet(&PID_ptr_M[9], LJ3L);
        PIDCompute(&PID_ptr_M[9]);
        FeedforwardAdd(&PID_ptr_M[9], LJ3des, true);

        //  std::cout<<"RJ3PID输出"<<PID_ptr_M[3].output<<'\n';
        //  std::cout<<"RJ3实际:"<<*TR_data[3]<<'\n';
        //  std::cout<<"RJ3期望"<<RJDES[3]<<'\n';
        //  std::cout<<"RJ3期望长度"<<RJ3Ldes<<'\n';
        //  std::cout<<"RJ3实际长度"<<RJ3L<<'\n';

        bipins.RJ4RJ5convert(RJDES[4], RJDES[5], RL4des, RL5des);
        bipins.RJ4RJ5convert(*TR_data[4], *TR_data[5], RL4real, RL5real);

        PIDSetpointSet(&PID_ptr_M[4], RL4des);
        PIDInputSet(&PID_ptr_M[4], RL4real);
        PIDCompute(&PID_ptr_M[4]);
        FeedforwardAdd(&PID_ptr_M[4], RL4des, true);

        PIDSetpointSet(&PID_ptr_M[5], RL5des);
        PIDInputSet(&PID_ptr_M[5], RL5real);
        PIDCompute(&PID_ptr_M[5]);
        FeedforwardAdd(&PID_ptr_M[5], RL5des, true);

        bipins.LJ4LJ5convert(LJDES[4], LJDES[5], LL4des, LL5des);
        bipins.LJ4LJ5convert(*TR_data[10], *TR_data[11], LL4real, LL5real);

        PIDSetpointSet(&PID_ptr_M[10], LL4des);
        PIDInputSet(&PID_ptr_M[10], LL4real);
        PIDCompute(&PID_ptr_M[10]);
        FeedforwardAdd(&PID_ptr_M[10], LL4des, true);

        PIDSetpointSet(&PID_ptr_M[11], LL5des);
        PIDInputSet(&PID_ptr_M[11], LL5real);
        PIDCompute(&PID_ptr_M[11]);
        FeedforwardAdd(&PID_ptr_M[11], LL5des, true);
        // PID_ptr_M[11].output = 0;

        int OutIndex[6] = {0, 1, 5, 6, 7, 11};

        // 创建一个布尔数组来标记
        bool is_out_index[12] = {false};
        for (int i = 0; i < 6; ++i)
        {
            is_out_index[OutIndex[i]] = true;
        }

        int c_idx = 0;
        int v_idx = 0;

        // PID_ptr_M[3].output = 1;

        // PID_ptr_M[4].output = 1;
        // PID_ptr_M[5].output = 1;
        //       PID_ptr_M[0].output=5*sin(M_PI*time);
        //  PID_ptr_M[8].output =  1+5*sin(M_PI*time);

        //  PID_ptr_M[9].output = 1+5*sin(M_PI*time);

        // PID_ptr_M[10].output =  1+5*sin(M_PI*time);
        // PID_ptr_M[11].output =  1+5*sin(M_PI*time);

        for (int i = 0; i < 12; ++i)
        {
            output1 = (uint16_t)((PID_ptr_M[i].output + 10) / 20 * 65535);
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

        // std::cout << "-------------------\n";

        // logger.Savedata();

        char sendBuf[128];

        // sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f\n", RJDES[0],*TR_data[0],*TR_data[2],*TR_data[3],
        // PID_ptr_M[2].alteredKp,PID_ptr_M[2].dispKi,rj2desl,PID_ptr_M[2].FF,PID_ptr_M[2].output);
        //   sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", *TR_data[0], *TR_data[1], *TR_data[2],
        //          *TR_data[3],*TR_data[4], *TR_data[5],*TR_data[6],*TR_data[7],*TR_data[8],*TR_data[9],*TR_data[10]
        //          ,*TR_data[11],RJDES[1],LJDES[1],gang0des,gang0real,gang1des,gang1real,lgang0des,lgang0real,lgang1des,lgang1real);

        //  sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", PID_ptr_M[1].Error1,PID_ptr_M[2].Error1,PID_ptr_M[3].Error1,PID_ptr_M[4].Error1,PID_ptr_M[5].Error1, PID_ptr_M[7].Error1,PID_ptr_M[8].Error1,PID_ptr_M[9].Error1,PID_ptr_M[10].Error1,PID_ptr_M[11].Error1,PID_ptr_M[4].input,PID_ptr_M[10].input);
        //   sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gang1des,rj2desl,RJ3Ldes,RL4des,RL5des,
        // lgang1des,lj2desl,LJ3des,LL4des,LL5des);
        //  sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", *TR_data[0], *TR_data[1], *TR_data[2],
        //          *TR_data[3], *TR_data[4], *TR_data[5], RJDES[0], RJDES[1], RJDES[2], RJDES[3], RJDES[4], RJDES[5], Deg[2], LDeg[2], (float)PressureA[2], (float)PressureB[2], PID_ptr_M[2].FF, PID_ptr_M[2].FFP, u, Ps);
        // sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",  Deg[0], Deg[1],Deg[2], LDeg[0], LDeg[1], LDeg[2],RJDES[0],RJDES[1],RJDES[2],RJDES[3],RJDES[4],RJDES[5],ifswing);
        // sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", RJDES[0], RJDES[1], RJDES[2],
        //         RJDES[3], RJDES[4], RJDES[5], LJDES[0], LJDES[1], LJDES[2], LJDES[3], LJDES[4], LJDES[5], *TR_data[0], *TR_data[1], *TR_data[2],
        //         *TR_data[3], *TR_data[4], *TR_data[5], *TR_data[6], *TR_data[7], *TR_data[8], *TR_data[9], *TR_data[10], *TR_data[11], PressureA[2], PressureB[2], PressureA[8], PressureB[8]);

        //   sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f\n",Deg[0],Deg[1],Deg[2],LDeg[0],LDeg[1],LDeg[2]);

        // sprintf(sendBuf, "d:%f,%f,%f,%f,%f,%f\n", AHRSData_Packet.Roll, AHRSData_Packet.Pitch, AHRSData_Packet.Heading,
        //         IMUData_Packet.accelerometer_x, IMUData_Packet.accelerometer_y, IMUData_Packet.accelerometer_z);
        sprintf(sendBuf,"d:%f,%f,%f,%f,%f,%f,%f,%f,%f\n",zmpscope[0],zmpscope[1],zmpscope[2],zmpscope[3],
        walkphase,dyx,dyy,rlegswingphase,rlegcontactphase);
        

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

    // if ((fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    // {
    //     perror("UART: Failed to open the UART device:ttyO1.\n");
    //     return -1;
    // }

    // // 获取原有串口配置
    // if (tcgetattr(fd1, &options1) < 0)
    // {
    //     return -1;
    // }

    // // 修改控制模式，保证程序不会占用串口
    // options1.c_cflag |= CLOCAL;

    // // 修改控制模式，能够从串口读取数据
    // options1.c_cflag |= CREAD;

    // // 不使用流控制
    // options1.c_cflag &= ~CRTSCTS;

    // // 设置数据位
    // options1.c_cflag &= ~CSIZE;
    // options1.c_cflag |= CS8;

    // // 设置奇偶校验位
    // options1.c_cflag &= ~PARENB;
    // options1.c_iflag &= ~INPCK;

    // // 设置停止位
    // options1.c_cflag &= ~CSTOPB;

    // // 设置最少字符和等待时间
    // options1.c_cc[VMIN] = 1;  // 读数据的最小字节数
    // options1.c_cc[VTIME] = 0; // 等待第1个数据，单位是10s

    // // 修改输出模式，原始数据输出
    // options1.c_oflag &= ~OPOST;
    // options1.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // // 设置波特率
    // cfsetispeed(&options1, B115200);
    // cfsetospeed(&options1, B115200);

    // // 清空终端未完成的数据
    // tcflush(fd1, TCIFLUSH);

    // // 设置新属性
    // if (tcsetattr(fd1, TCSANOW, &options1) < 0)
    // {
    //     return -1;
    // }

    // control设置
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

    DesiredStateCommand *desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData *_controlData = new ControlFSMData;
    _controlData->_biped = &robot;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = Legctrlptr;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = IOptr;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM *_FSMController = new FSM(_controlData);

    // PID传参
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

    off_bytes_0x6010[0] = ecrt_slave_config_reg_pdo_entry(sc1, 0x6010, 1, domain, &off_bits_0x6010[0]); // rj2A pressure
    printf("off_bytes_0x6010_value=0x%x %x\n", off_bytes_0x6010[0], off_bits_0x6010[0]);
    if (off_bytes_0x6010[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6010[1] = ecrt_slave_config_reg_pdo_entry(sc1, 0x6010, 2, domain, &off_bits_0x6010[1]); // rj2B pressure
    printf("off_bytes_0x6010_value=0x%x %x\n", off_bytes_0x6010[1], off_bits_0x6010[1]);
    if (off_bytes_0x6010[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6010[2] = ecrt_slave_config_reg_pdo_entry(sc1, 0x6010, 3, domain, &off_bits_0x6010[2]); // lj2A pressure
    printf("off_bytes_0x6010_value=0x%x %x\n", off_bytes_0x6010[2], off_bits_0x6010[2]);
    if (off_bytes_0x6010[2] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6010[3] = ecrt_slave_config_reg_pdo_entry(sc1, 0x6010, 4, domain, &off_bits_0x6010[3]); // lj2B pressure
    printf("off_bytes_0x6010_value=0x%x %x\n", off_bytes_0x6010[3], off_bits_0x6010[3]);
    if (off_bytes_0x6010[3] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6010_1[0] = ecrt_slave_config_reg_pdo_entry(sc2, 0x6010, 1, domain, &off_bits_0x6010_1[0]); // PS pressure
    printf("off_bytes_0x6010_value=0x%x %x\n", off_bytes_0x6010_1[0], off_bits_0x6010_1[0]);
    if (off_bytes_0x6010_1[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x6010_1[1] = ecrt_slave_config_reg_pdo_entry(sc2, 0x6010, 2, domain, &off_bits_0x6010_1[1]); // P0 pressure
    printf("off_bytes_0x6010_value=0x%x %x\n", off_bytes_0x6010_1[1], off_bits_0x6010_1[1]);
    if (off_bytes_0x6010_1[1] < 0)
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

    off_bytes_0x7030_1 = ecrt_slave_config_reg_pdo_entry(sc2, 0x7030, 1, domain, &off_bits_0x7030_1);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x7030_1, off_bits_0x7030_1);
    if (off_bytes_0x7030_1 < 0)
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
    printf("off_bytes_0x7010_value=0x%x %x\n", off_bytes_0x7010[0], off_bits_0x7010[0]);
    if (off_bytes_0x7010[0] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[1] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 2, domain, &off_bits_0x7010[1]);
    printf("off_bytes_0x7010_value=0x%x %x\n", off_bytes_0x7010[1], off_bits_0x7010[1]);
    if (off_bytes_0x7010[1] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[2] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 3, domain, &off_bits_0x7010[2]);
    printf("off_bytes_0x7010_value=0x%x %x\n", off_bytes_0x7010[2], off_bits_0x7010[2]);
    if (off_bytes_0x7010[2] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[3] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 4, domain, &off_bits_0x7010[3]);
    printf("off_bytes_0x7010_value=0x%x %x\n", off_bytes_0x7010[3], off_bits_0x7010[3]);
    if (off_bytes_0x7010[3] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[4] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 5, domain, &off_bits_0x7010[4]);
    printf("off_bytes_0x7010_value=0x%x %x\n", off_bytes_0x7010[4], off_bits_0x7010[4]);
    if (off_bytes_0x7010[4] < 0)
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    off_bytes_0x7010[5] = ecrt_slave_config_reg_pdo_entry(sc3, 0x7010, 6, domain, &off_bits_0x7010[5]);
    printf("off_bytes_0x7010_value=0x%x %x\n", off_bytes_0x7010[5], off_bits_0x7010[5]);
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
    tv.it_interval.tv_usec = 20000; // 10 ms  运行一次signal_handler
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
    // filter.set(1 / sampletime, 50);
    PID_ptr_M = new PIDControl[12];
    FPID_ptr = new fuzzypid[12];

    /*
   PIDK: 0.8,0.2,0,-0.1
   PIDK0: 0.22,0.02,0,-0.12
   PIDK1:0.22, 0.03, 0,-0.13,
   */
    float gain = 1;
    PIDInit(&PID_ptr_M[0], 0.25 * gain, 0.018 * gain, 0, -0.12, controltime, -9, 9, AUTOMATIC, REVERSE);
    // PIDInit(&PID_ptr_M[1] 0.065, 0, 0.0035, controltime, -10, 10, AUTOMATIC, DIRECT);
    // PIDInit(&PID_ptr_M[2], 0.048, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);

    PIDInit(&PID_ptr_M[1], 0.25 * gain, 0.012 * gain, 0, -0.12, controltime, -9, 9, AUTOMATIC, REVERSE);
    /*
   PIDK: 0.065,0.024,0,-0.02
    PIDK1: 0.097,0.04
   */
    PIDInit(&PID_ptr_M[2], 0.08, 0.06, 0, 0.02, controltime, -9, 9, AUTOMATIC, DIRECT);

    // FPID_ptr[2].fuzzypid_init(0.09, 0.025, 0.02, controltime, F_DIRECT);
    // FPID_ptr[2].Fuzzyset(0.01, -0.01, 0.005, -0.005, 2.5, -2.5);
    /*
    PIDK: 0.06,0.004,0,0.0811
    PIDKL:0.045, 0.016, 0, 0.027
    */
    PIDInit(&PID_ptr_M[3], 0.06, 0.02, 0, 0.02, controltime, -9, 9, AUTOMATIC, DIRECT);
    /*
     PIDK:0.35, 0.05, 0, -0.05 REVERSE 单关节0.25Hz
     */
    PIDInit(&PID_ptr_M[4], 0.35, 0.05, 0, -0.03, controltime, -9, 9, AUTOMATIC, REVERSE);
    /*
    PIDK: 1.0,0.08,0,0.1 DIRECT
    */
    PIDInit(&PID_ptr_M[5], 1.2, 0.07, 0, 0.08, controltime, -9, 9, AUTOMATIC, DIRECT);

    /*
    PIDK: 0.8,0.2,0,-0.1
    PIDK0: 0.25, 0.02, 0, 0.1
    PIDK1:0.25, 0.02, 0, 0.1
    */
    PIDInit(&PID_ptr_M[6], 0.28, 0.018, 0, 0.08, controltime, -9, 9, AUTOMATIC, DIRECT);
    // PIDInit(&PID_ptr_M[1], 0.065, 0, 0.0035, controltime, -10, 10, AUTOMATIC, DIRECT);
    // PIDInit(&PID_ptr_M[2], 0.048, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);

    PIDInit(&PID_ptr_M[7], 0.28, 0.018, 0, 0.08, controltime, -9, 9, AUTOMATIC, DIRECT);
    /*
    PIDK: 0.065,0.024,0,-0.02
    0.1, 0.06, 0, -0.02,
   */
    PIDInit(&PID_ptr_M[8], 0.09, 0.06, 0, -0.02, controltime, -9, 9, AUTOMATIC, REVERSE);
    /*
    PIDK: 0.06,0.004,0,0.0811
    */
    PIDInit(&PID_ptr_M[9], 0.05, 0.014, 0, 0.027, controltime, -9, 9, AUTOMATIC, DIRECT);
    /*
    PIDK: 0.4,0,0,0.02
    */
    PIDInit(&PID_ptr_M[10], 0.35, 0.05, 0, -0.03, controltime, -9, 9, AUTOMATIC, REVERSE);
    /*
    PIDK: 1.0,0.08,0,0.1
    */
    PIDInit(&PID_ptr_M[11], 1, 0.08, 0, 0.08, controltime, -9, 9, AUTOMATIC, DIRECT);

    printf("Started.\n");

    Biped bipedinstance;

    bipedinstance.sampletime = 0.01;

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
            //  std::cout << "时间: " << time << '\n';
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
