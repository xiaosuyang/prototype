#include "../../include/FSM/FSMState.h"

FSMState::FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr) : _data(data), _stateName(stateName), _stateNameStr(stateNameStr), cmdPanel()
{
    cmd = cmdPanel.getUserCmd();
    _lowCmd = _data->_lowCmd;
    _lowState = _data->_lowState;
}

FSMStateName FSMState::checkTransition()
{
    switch (cmd)
    {
    case L1_Y:
        return DEBUG;

    case NONE:
        return INVALID;

    default:
        return INVALID;
    }
}
