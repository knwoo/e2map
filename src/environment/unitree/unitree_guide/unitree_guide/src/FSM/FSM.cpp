/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);

#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
    _nm.getParam("/stand_wait_count", _stand_wait_count);
    _nm.getParam("/move_base_wait_count", _move_base_wait_count);
    _nm.getParam("/reset_topic", _reset_topic);
    _resetSub = _nm.subscribe(_reset_topic, 10, &FSM::resetCallback, this);
    _resetCompletePub = _nm.advertise<std_msgs::Bool>("/reset_complete_topic", 10);
    
#endif  // COMPILE_WITH_MOVE_BASE
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::resetCallback(const std_msgs::Bool& msg) {
    _reset = msg.data;
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
    _count = 0;
    _reset = false;
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        _ctrlComp->ioInter->setPassive();
    }
    if (_reset) {
        _currentState->exit();
        _currentState = _stateList.fixedStand;
        _currentState->enter();
        _count = _stand_wait_count;
        _reset = false;
        std_msgs::Bool comp_msg;
        comp_msg.data = true;
        _resetCompletePub.publish(comp_msg);
    }

    if (_currentState != _stateList.fixedStand && _currentState != _stateList.moveBase && _count > _stand_wait_count) {
        _currentState->exit();
        _currentState = _stateList.fixedStand;
        _currentState->enter();
    } 
    else if (_currentState != _stateList.moveBase && _count > _stand_wait_count + _move_base_wait_count) {
        _currentState->exit();
        _currentState = _stateList.moveBase;
        _currentState->enter();
    }

    if (_currentState != _stateList.moveBase) {
        _count += 1;
    }
        
    _currentState->run();
    

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}