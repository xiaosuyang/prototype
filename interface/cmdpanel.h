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
};

class CmdPanel{
public:
    CmdPanel(){}
    ~CmdPanel(){}
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