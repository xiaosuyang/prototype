#ifndef CMDPANEL_H
#define CMDPANEL_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include "enumclass.h"


struct UserValue
{
    float direction=0;
    bool Starttime=false;
    bool Settime=false;

    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    float vx; // vx in body frame
    float vy; // vy in body frame
    float turn_rate;

    UserValue()
    {
        setZero();
    }
    void setZero()
    {
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
        vx = 0;
        vy = 0;
        turn_rate = 0;
    }
};

class CmdPanel{
public:
    CmdPanel(){}
    ~CmdPanel(){}
    UserCommand getUserCmd() const
    {
        return userCmd;
    }
    UserValue uservalue;

protected:
    virtual void *run(void *arg){};
    UserCommand userCmd;
    
};

class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void *runKeyBoard(void *arg);
    void *run(void *arg);
    void changeValue();
    UserCommand checkCmd();

    pthread_t _tid;
    float sensitivityUp = 0.5;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    char _c;
};







#endif