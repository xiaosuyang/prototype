#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include "../ControlFSMData.h"
#include "../utility/cppTypes.h"
#include "../../interface/enumclass.h"
#include "../../interface/cmdpanel.h"
#include "../../interface/LowLevelCmd.h"
#include "../../interface/LowlevelState.h"

class FSMState
{
    public:
        FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkTransition();


        FSMStateName _stateName;
        std::string _stateNameStr;

    protected:
        ControlFSMData *_data;
        FSMStateName _nextStateName;

        LowlevelCmd *_lowCmd;
        LowlevelState *_lowState;
        UserValue _userValue;

    private:
        CmdPanel cmdPanel;
        UserCommand cmd;
};

#endif // FSMSTATE_H