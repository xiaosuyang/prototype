/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */


#ifndef PROJECT_STATE_ESTIMATOR_CONTAINER_H
#define PROJECT_STATE_ESTIMATOR_CONTAINER_H

#pragma once

#include "LegController.h"
#include "../interface/LowlevelState.h"

/*!
 * Result of state estimation
 */

struct StateEstimate {
    Vec4<double> contactEstimate;
    Vec3<double> position;
    Vec3<double> vBody;
    Quat<double> orientation;
    //躯干坐标系下角速度
    Vec3<double> omegaBody;
    RotMat<double> rBody;
    Vec3<double> rpy;

    //world frame 下角速度
    Vec3<double> omegaWorld;
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;
};


/*!
 * input for state estimation
 */ 
struct StateEstimatorData {
    StateEstimate* result;
    LowlevelState* lowState;
    LegControllerData* legControllerData;
};

/*!
 * All Estimators inherit from this class
 */ 
class GenericEstimator{
  public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData data) {_stateEstimatorData = data;};

    // virtual ~GenericEstimator() = default;
    virtual ~GenericEstimator(){};
    StateEstimatorData _stateEstimatorData;
};

/*!
 * Main State Estimator class
 * Contains all GenericEstimator
 */ 
class StateEstimatorContainer {
  public:
    // Constructor
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateEstimatorContainer(LowlevelState *_lowState,
                            LegControllerData *_legControllerData,
                            StateEstimate *stateEstimate){
        _data.lowState = _lowState;
        _data.legControllerData = _legControllerData;
        _data.result = stateEstimate;
    }
    // deconstructor
    ~StateEstimatorContainer() {
        // for (auto estimator : _estimators) {
        //     delete estimator;
        // }
        int i = 0;
        while (i < 10)
        {
            // if (_estimators[i] != nullptr)
            if (_estimators[i] != NULL)
            {
                delete _estimators[i];
                // _estimators[i] = nullptr;
                _estimators[i] = NULL;
            }
            i++;
        }
    }
    // run estimator
    void run()
    {
        int i = 0;
        while (i < 10)
        {
            // if (_estimators[i] != nullptr)
            if (_estimators[i] != NULL)
            {
                _estimators[i]->run();
            }
            i++;
        }
    }

    // get result
    const StateEstimate&  getResult() {return *_data.result;}

    // add estimator of given type
    template <typename EstimatorToAdd>
    void addEstimator(){
        std::cout << "add estimator" << std::endl;
        GenericEstimator* estimator = new EstimatorToAdd();
        estimator->setData(_data);
        estimator->setup();
        estimator->run();
        // _estimators.push_back(estimator);
    }

    // remove estimator of given type
    // template <typename EstimatorToRemove>
    // void removeEstimator() {
    //     int nRemoved = 0;
    //     _estimators.erase(
    //         std::remove_if(_estimators.begin(), _estimators.end(),
    //                        [&nRemoved](GenericEstimator* e){
    //                            if (dynamic_cast<EstimatorToRemove*>(e)){
    //                                delete e;
    //                                nRemoved++;
    //                                return true;
    //                             } else {
    //                                 return false;
    //                             }
    //                        }),
    //         _estimators.end());
    // }

    // // remove all estimators
    // void removeAllEstimators() {
    //     for (auto estimator : _estimators) {
    //         delete estimator;
    //     }
    //     _estimators.clear();
    // }
  private:
    GenericEstimator* _estimators[10];
    Vec4<double> _phase;
    StateEstimatorData _data;
};


#endif