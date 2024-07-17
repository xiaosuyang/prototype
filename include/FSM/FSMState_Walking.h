// #ifndef WALKING_H
// #define WALKING_H

// #include "FSMState.h"
// #include "../../ConvexMPC/ConvexMPCLocomotion.h"
// #include "logdata/logdata.h"

// class FSMState_Walking: public FSMState
// {
//     public:
//         FSMState_Walking(ControlFSMData *data,ConvexMPCLocomotion& cmpc);
//         ~FSMState_Walking(){}
//         void enter();
//         void run();
//         void exit();
//         FSMStateName checkTransition();
    
//     private:
//         ConvexMPCLocomotion& Cmpc;
//         int counter;
//         Vec3<double> v_des_body;
//         double turn_rate = 0;
//         double pitch, roll;
//         DataLogger& logger=DataLogger::GET();
// };

// #endif