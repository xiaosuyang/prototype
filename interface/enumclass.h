#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>



enum UserCommand{
    // EXIT,
    NONE,
    START,      // walking
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // pushing
    L2_Y,       // probe
    L1_X,       // QPStand 
    L1_A,      
    L1_Y       
};

enum FSMMode{
    NORMAL,
    CHANGE
};

// enum class FSMStateName{
enum  FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    PDSTAND,
    QPSTAND,
    WALKING,
    PUSHING,
    PROBE,
    DEBUG,       // slam
};


#endif  // ENUMCLASS_H