// #include "../../include/FSM/FSMState_Walking.h"

// FSMState_Walking::FSMState_Walking(ControlFSMData *data,ConvexMPCLocomotion& cmpc)
//                  :FSMState(data, FSMStateName::WALKING, "walking"),
//                   Cmpc(cmpc) {}

// template<typename T0, typename T1, typename T2>
// T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
// 	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
// }

// void FSMState_Walking::enter()
// {
//     v_des_body << 0, 0, 0;
//     pitch = 0;
//     roll = 0;
//     _data->_interface->zeroCmdPanel();
//     counter = 0;
//     _data->_desiredStateCommand->firstRun = true;
//     _data->_stateEstimator->run(); 
//     _data->_legController->zeroCommand();
//     Cmpc.firstRun = true;
// }

// void FSMState_Walking::run()
// {
//     _data->_legController->updateData(_data->_lowState);
//     _data->_stateEstimator->run(); 
//     _userValue = _data->_lowState->userValue;
//     logger.vx_ly=_userValue.ly;
//     v_des_body[0] = (double)_userValue.ly;
//     v_des_body[1] = (double)invNormalize(_userValue.rx, -0.25, 0.25);
//     turn_rate = (double)invNormalize(_userValue.lx, -0.5, 0.5);
//     // std::cout << "vx vy " << v_des_body[0] << " " << v_des_body[1] << std::endl;
//     _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    
//     Cmpc.setGaitNum(2); // 2 for walking
//     Cmpc.run(*_data);

//     _data->_legController->updateCommand(_data->_lowCmd);  
// }

// void FSMState_Walking::exit()
// {  
//     //auto staters=_data->_stateEstimator->getResult();  
//     counter = 0; 
//     _data->_interface->zeroCmdPanel();
// }

// FSMStateName FSMState_Walking::checkTransition()
// {
//     if(_lowState->userCmd == UserCommand::L2_B){
//         return FSMStateName::PASSIVE;
//     }
//     else{
//         return FSMStateName::WALKING;
//     }
// }