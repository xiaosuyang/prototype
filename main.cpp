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
#include "datalogger.h"
#include "Eigen/Dense"
#include "biped.h"
/****************************************************************************/

// Application parameters
#define FREQUENCY 100
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS 1

#define BIGNUM 100000000.0
PIDControl *PID_ptr_M; // 矢状面3关节，j2,j3,j4;

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
static unsigned int off_bytes_0x6000[2] = {1};
static unsigned int off_bits_0x6000[2] = {1};

static unsigned int off_bytes_0x6000_1[2] = {1};
static unsigned int off_bits_0x6000_1[2] = {1};

static unsigned int off_bytes_0x7011[3] = {1};
static unsigned int off_bits_0x7011[3] = {1};

static unsigned int counter = 0;

static unsigned int blink = 0x00;
static uint8_t *domain_pd = NULL;

#define BusCouplerPos1 0, 0

#define BusCouplerPos2 0, 2

#define BusCouplerPos3 0, 1

#define TI_AM3359ICE 0x01222222, 0x00020310
#define Valve 0x00000b95, 0x00020130

#include "cstruct.h"

int fd = -1;
struct sockaddr_in saddr;
DataLogger &logger = DataLogger::GET();

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

void cyclic_task(Biped &bipins, float time)
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

        double LEGlength = bipins.calfLinkLength + bipins.thighLinkLength + bipins.toeLinkLength;
        Vec6<double> RjQ;
        Vec3<double> P;
        float Deg[3];
        P.setZero();
        P(2) = -1 * LEGlength - 0.07 * LEGlength * (cos(0.2 * M_PI * time) - 1);
        Deg[0] = (14 * cos(0.2 * M_PI * time) - 14);
      //  Deg[2] = 0;
       // Deg[1] = 0;
        Deg[2] = (18 * cos(0.2 * M_PI * time) - 18);
        Deg[1] = -1 * (Deg[0] + Deg[2]);
        // P(2)*=-1;
        bipins.ComputeIK(RjQ, P);

        std::cout << "坐标\n"
                  << P << '\n';
        float RJDES[4];
        RJDES[0] = Deg[0];
        RJDES[1] = Deg[1];
        RJDES[2] = 0;
        RJDES[3]=0;

       // unsigned long ssi[4];
        std::vector<unsigned long> ssi;
        unsigned long SSI[12];
        for(int i=0;i<12;i++) SSI[i]=0;
  
        uint16_t output1 = 0;
        SSI[0]=EC_READ_U32(domain_pd + off_bytes_0x6000[0]);
        SSI[1]=EC_READ_U32(domain_pd + off_bytes_0x6000[1]);
        SSI[2]=EC_READ_U32(domain_pd + off_bytes_0x6000_1[0]);
        SSI[3]=EC_READ_U32(domain_pd + off_bytes_0x6000_1[1]);
        std::cout<<"AFTER READ\n";
        float * TR_data[12];
        float fSSI[12];
        for(int i=0;i<12;i++)
        {
            // TR_data[i]=(float *)SSI[i];
            fSSI[i]=(float)SSI[i];
            TR_data[i] = &fSSI[i];
            if (TR_data[i] == nullptr)
            {
                // 处理错误情况
                std::cerr << "Error: SSI[" << i << "] is null." << std::endl;
                exit(1);
            }
        }
        

        float Ldes = bipins.RJ2convert(RJDES[0]);
        float Lreal = bipins.RJ2convert(*TR_data[0]);

        float gang0des,gang1des;
        float gang0real,gang1real;
        bipins.RJ0RJ1convert(RJDES[2],RJDES[3],gang0des,gang1des);
        bipins.RJ0RJ1convert(*TR_data[2],*TR_data[3],gang0real,gang1real);
        
        PIDSetpointSet(&PID_ptr_M[0], Ldes);//RJ2
        PIDInputSet(&PID_ptr_M[0], Lreal);
     

        PIDSetpointSet(&PID_ptr_M[1], RJDES[1]);
        PIDInputSet(&PID_ptr_M[1],*TR_data[1]);//RJ3

        PIDSetpointSet(&PID_ptr_M[2], gang0des);
        PIDInputSet(&PID_ptr_M[2],gang0real);//gang0

        PIDSetpointSet(&PID_ptr_M[3], gang1des);
        PIDInputSet(&PID_ptr_M[3],gang1real);//gang0

     std::cout<<"AFTER PID\n";
        for (int i = 0; i < 4; i++)
        {
            PIDCompute(&PID_ptr_M[i]);
            EC_WRITE_U16(domain_pd + off_bytes_0x7011[i], output1);
            printf("Volage1: value=0x%x\n", EC_READ_U16(domain_pd + off_bytes_0x7011[i]));
        }


        counter = 2;

        std::cout << "-------------------\n";

        logger.Savedata();
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
    
    // std::vector<float> vectest{1,2,3,4},vectest1{5,6,7,8};
    
    // vectest.insert(vectest.end(),vectest1.begin(),vectest1.end());
    
    // auto testpair=std::make_pair(1,2);

    float Kp = 0.01, Kd = 0;
    if (argc < 2)
        std::cout << "No Kp" << std::endl;
    else if (argc == 2)
        Kp = atof(argv[1]);
    else
    {
        Kp = atof(argv[1]);
        Kd = atof(argv[2]);
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
              master, BusCouplerPos2, TI_AM3359ICE)))
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

    const float controltime = 0.02, sampletime = 0.01;
    // pid_ptr = (PIDControl*)malloc(sizeof(PIDControl));
    // PIDInit(pid_ptr,Kp,Kd,0,0.02,-10,10,AUTOMATIC,DIRECT);
    PID_ptr_M = new PIDControl[12];
    PIDInit(&PID_ptr_M[0], Kp, Kd, 0, controltime, -10, 10, AUTOMATIC, DIRECT);
   // PIDInit(&PID_ptr_M[1], 0.065, 0, 0.0035, controltime, -10, 10, AUTOMATIC, DIRECT);
    //PIDInit(&PID_ptr_M[2], 0.048, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);

    PIDInit(&PID_ptr_M[1], 0.05, 0, 0.0035, controltime, -10, 10, AUTOMATIC, DIRECT);
    PIDInit(&PID_ptr_M[2], 0.03, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);
    PIDInit(&PID_ptr_M[3], 0.03, 0, 0, controltime, -10, 10, AUTOMATIC, DIRECT);

    printf("Started.\n");

    Biped bipedinstance;
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

            cyclic_task(bipedinstance, time);
            user_alarms++;
        }
        // std::cout<<"Keyboard Value\n"<<cmd_ptr->uservalue.direction<<'\n';
    }

    delete[] PID_ptr_M;
    delete cmdptr;
    return 0;
}

/****************************************************************************/
