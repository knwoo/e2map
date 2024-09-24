#include "dynamics.h"


State Dynamics::UpdateState(const State& prevState, const Action& action){
    State newState(0,0,0,0);

    double cos = std::cos(prevState.yaw);
    double sin = std::sin(prevState.yaw);
    newState.x = prevState.x + (action.vx * cos + action.wz * _ICR * sin) * _DT;
    newState.y = prevState.y + (action.vx * sin - action.wz * _ICR * cos) * _DT;
    newState.yaw = prevState.yaw + action.wz * _DT;
    return newState;
}

void Dynamics::SetConfig(const ros::NodeHandle& nh){
    nh.getParam(ros::this_node::getName() + "/NX", _NX);
    nh.getParam(ros::this_node::getName() + "/NU", _NU);
    nh.getParam(ros::this_node::getName() + "/T", _T);
    nh.getParam(ros::this_node::getName() + "/DT", _DT);
    nh.getParam(ros::this_node::getName() + "/ICR", _ICR);
    nh.getParam(ros::this_node::getName() + "/maxLinVel", _maxLinVel);
    nh.getParam(ros::this_node::getName() + "/maxAngVel", _maxAngVel);
}

void Dynamics::PrintConfig(){
    std::cout << "Dynamics : " << std::endl;
    std::cout << "\tNX : " << _NX << std::endl;
    std::cout << "\tNU : " << _NU << std::endl;
    std::cout << "\tT : " << _T << std::endl;

    std::cout << "\tDT : " << _DT << std::endl;
    std::cout << "\tICR : " << _ICR << std::endl;
    std::cout << "\tmaxLinVel : " << _maxLinVel << std::endl;
    std::cout << "\tmaxAngVel : " << _maxAngVel << std::endl;
}

int Dynamics::GetNx(){
    return _NX;
}

int Dynamics::GetNu(){
    return _NU;
}

double Dynamics::GetT(){
    return _T;
}

double Dynamics::GetDT(){
    return _DT;
}

double Dynamics::GetMaxLinVel(){
    return _maxLinVel;
}

double Dynamics::GetMaxAngVel(){
    return _maxAngVel;
}

void Dynamics::SetT(const int& T){
    _T = T;
}

void Dynamics::SetDT(const double& DT){
    _DT = DT;
}