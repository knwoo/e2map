#include "action.h"


std::vector<double> Action::toVector(){
    std::vector<double> vec{vx, wz};
    return vec;
}

std::vector<double> Action::toVector() const{
    std::vector<double> vec{vx, wz};
    return vec;
}

Action Action::operator+(const double& num) const{
    Action temp(vx + num, wz + num);
    return temp;
}

Action Action::operator-(const double& num) const{
    Action temp(vx - num, wz - num);
    return temp;
}

Action Action::operator*(const double& num) const{
    Action temp(vx * num, wz * num);
    return temp;
}

Action Action::operator/(const double& num) const{
    Action temp(vx / num, wz / num);
    return temp;
}

Action Action::operator+(const Action& action) const{
    Action temp(vx + action.vx, wz + action.wz);
    return temp;
}

Action Action::operator-(const Action& action) const{
    Action temp(vx - action.vx, wz - action.wz);
    return temp;
}

Action Action::operator*(const Action& action) const{
    Action temp(vx * action.vx, wz * action.wz);
    return temp;
}

Action Action::operator/(const Action& action) const{
    Action temp(vx / action.vx, wz / action.wz);
    return temp;
}

Action& Action::operator=(const Action& action){
    vx = action.vx;
    wz = action.wz;
    return *this;
}

Action& Action::operator=(const double& num){
    vx = num;
    wz = num;
    return *this;
}

Action& Action::operator+=(const double& num){
    (*this) = (*this) + num;
    return *this;
}

Action& Action::operator-=(const double& num){
    (*this) = (*this) - num;
    return *this;
}

Action& Action::operator*=(const double& num){
    (*this) = (*this) * num;
    return *this;
}

Action& Action::operator/=(const double& num){
    (*this) = (*this) / num;
    return *this;
}

Action& Action::operator+=(const Action& action){
    (*this) = (*this) + action;
    return *this;
}

Action& Action::operator-=(const Action& action){
    (*this) = (*this) - action;
    return *this;
}

Action& Action::operator*=(const Action& action){
    (*this) = (*this) * action;
    return *this;
}

Action& Action::operator/=(const Action& action){
    (*this) = (*this) / action;
    return *this;
}

std::ostream& operator<<(std::ostream& os, const Action& action){
    os << "[" << action.vx << ", " << action.wz << "]";
    return os;
}