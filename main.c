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
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>
#include<arpa/inet.h>
#include"control/pid_controller.h"
/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 100
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS 1


#define BIGNUM 100000000.0
PIDControl* pid_ptr;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};

static ec_slave_config_t *sc;
static ec_slave_config_state_t sc_ana_in_state = {};


// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

/****************************************************************************/

// process data
// static uint8_t *domain_pd = NULL;

#define BusCouplerPos 0, 0

// #define TI_AM3359ICE 0x00000009, 0x26483052

// // offsets for PDO entries
static unsigned int off_bytes_0x6000;
static unsigned int off_bits_0x6000;
// static unsigned int off_bytes_0x7010;
// static unsigned int off_bits_0x7010;

static unsigned int counter = 0;
static unsigned int blink = 0x00;
static uint8_t *domain_pd = NULL;

#define BusCouplerPos1 0, 0

#define BusCouplerPos2 0, 1

#define TI_AM3359ICE 0x00000b95, 0x00020310

// offsets for PDO entries
// static unsigned int off_bytes_cm_1[2]={-1};
// static unsigned int off_bits_cm_1[2]={-1};
// static unsigned int off_bytes_cm_2[2]={-1};
// static unsigned int off_bits_cm_2[2]={-1};
// static unsigned int off_bytes_target_angle[2]={-1};
// static unsigned int off_bits_target_angle[2]={-1};
// static unsigned int off_bytes_ssi[2]={-1};
// static unsigned int off_bits_ssi[2]={-1};
// static unsigned int off_bytes_ain_1[2]={-1};
// static unsigned int off_bits_ain_1[2]={-1};
// static unsigned int off_bytes_ain_2[2]={-1};
// static unsigned int off_bits_ain_2[2]={-1};
// static unsigned int off_bytes_ain_3[2]={-1};
// static unsigned int off_bits_ain_3[2]={-1};
// static unsigned int off_bytes_ain_4[2]={-1};
// static unsigned int off_bits_ain_4[2]={-1};
// static unsigned int off_bytes_ain_5[2]={-1};
// static unsigned int off_bits_ain_5[2]={-1};
// static unsigned int off_bytes_ain_6[2]={-1};
// static unsigned int off_bits_ain_6[2]={-1};
// static unsigned int off_bytes_ain_7[2]={-1};
// static unsigned int off_bits_ain_7[2]={-1};
// static unsigned int off_bytes_ain_8[2]={-1};
// static unsigned int off_bits_ain_8[2]={-1};

// static unsigned int counter = 0;
// static unsigned int blink = 0x00;

/* Master 0, Slave 0, "SSC-Device"
 * Vendor ID:       0x00000009
 * Product code:    0x26483052
 * Revision number: 0x00020111
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 2, slave_0_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_0_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 1, slave_0_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_0_pdo_entries + 4}, /* read ain of moudle */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 2, EC_WD_DISABLE},
    {0xff}
};




int fd=-1;
struct sockaddr_in saddr;

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

void check_slave_config_states(void)
{
    // printf("check_slave_config_states\n");
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc, &s);

    if (s.al_state != sc_ana_in_state.al_state)
        printf("AnaIn1: State 0x%02X.\n", s.al_state);

    if (s.online != sc_ana_in_state.online)
        printf("AnaIn1: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_ana_in_state.operational)
        printf("AnaIn1: %soperational.\n",
               s.operational ? "" : "Not ");

    sc_ana_in_state = s;
}


void cyclic_task()
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

        // unsigned long ssi;
        // ssi=EC_READ_U32(domain_pd + off_bytes_ssi );
        // float *Tr_data;
        // Tr_data=(float *)&ssi;
        // printf("SSI: value2=%f\n",*Tr_data);
       


        unsigned long ssi;
        ssi=EC_READ_U32(domain_pd + off_bytes_0x6000);
       // float *trdata;
        float *Tr_data;
        Tr_data=(float *)&ssi;

        PIDSetpointSet(pid_ptr,0.0);
        PIDInputSet(pid_ptr,*Tr_data);
        PIDCompute(pid_ptr);
        float output=pid_ptr->output;

        printf("SSI: value2=%f\n",*Tr_data);
        counter = 10;
    }


    // calculate new process data
    blink++;

    // check for master state (optional)
    check_master_state();

    // EC_WRITE_U16(domain_pd + off_bytes_led_1, 1 << 2);
    // printf("LED1: value=0x%x\n", EC_READ_U16(domain_pd + off_bytes_led_1));

    //  send process data
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
    
 

    struct sigaction sa;
    struct itimerval tv;

    master = ecrt_request_master(0);
    if (!master)
        return -1;
    // 建立一个域，参数是master
    domain = ecrt_master_create_domain(master);
    if (!domain)
        return -1;

        // ⽤宏定义配置master和slave 数字信号输⼊的从站/供应商ID产品代码
#if CONFIGURE_PDOS
    if (!(sc = ecrt_master_slave_config(
              master, BusCouplerPos, TI_AM3359ICE)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }



    // 设置PDOS，利⽤sc、EC_END、对⻬信息
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

 
    off_bytes_0x6000 = ecrt_slave_config_reg_pdo_entry(sc, 0x6000, 1, domain, &off_bits_0x6000);
    printf("off_bytes_0x6000_value=0x%x %x\n", off_bytes_0x6000, off_bits_0x6000);
    if (off_bytes_0x6000 < 0)
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
    tv.it_interval.tv_usec = 1000000 / FREQUENCY;
    tv.it_value.tv_sec = 0;
    tv.it_value.tv_usec = 2000;
    if (setitimer(ITIMER_REAL, &tv, NULL))
    {
        fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
        return 1;
    }

    pid_ptr = (PIDControl*)malloc(sizeof(PIDControl));
    PIDInit(pid_ptr,1,0,0,0.1,-BIG_NUM,BIG_NUM,AUTOMATIC,DIRECT);

    printf("Started.\n");
    while (1)
    {
        pause();
        while (sig_alarms != user_alarms)
        {
            cyclic_task();
            user_alarms++;
        }
    }
    free(pid_ptr);
    return 0;
}

/****************************************************************************/