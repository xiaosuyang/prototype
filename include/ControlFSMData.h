#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "DesiredCommand.h"
#include "LegController.h"
#include "biped.h"
#include "../interface/LowLevelCmd.h"
#include "../interface/LowlevelState.h"
#include "../interface/IOInterface.h"
#include "StateEstimatorContainer.h"
#include <memory>
//#include "../robotwrapper/robotwrapper.h"

struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Biped* _biped;
  
  StateEstimatorContainer* _stateEstimator;

  LegController* _legController;

  //std::shared_ptr<RobotWrapper> _RobotWrapperptr;

  DesiredStateCommand* _desiredStateCommand;

  IOInterface* _interface;

  LowlevelCmd* _lowCmd;

  LowlevelState* _lowState;

  void sendRecv(){
    _interface->sendRecv(_lowCmd, _lowState);
  }
};


#endif  // CONTROLFSM_H