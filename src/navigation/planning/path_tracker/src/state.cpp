#include "state.h"


double State::GetDistance(const State& state) const {
    double distance = std::sqrt(std::pow(x - state.x, 2) + std::pow(y - state.y, 2) + std::pow(z - state.z, 2));
    return distance;
}

double State::GetDistance(const geometry_msgs::Point& pt) const {
    double distance = std::sqrt(std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2) + std::pow(z - pt.z, 2));
    return distance;
}

double State::GetDistance(const PathStamp& stamp) const {
    double distance = std::sqrt(std::pow(x - stamp.x, 2) + std::pow(y - stamp.y, 2) + std::pow(z - stamp.z, 2));
    return distance;
}

double State::GetDistance(const double& p, const double& q, const double& r) const {
    double distance = std::sqrt(std::pow(x - p, 2) + std::pow(y - q, 2) + std::pow(z - r, 2));
    return distance;
}

std::vector<double> State::toVector() {
    std::vector<double> vec{x, y, v, yaw};
    return vec;
}

std::vector<double> State::toVector() const {
    std::vector<double> vec{x, y, v, yaw};
    return vec;
}

geometry_msgs::Point State::toGeoPoint() {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}

geometry_msgs::Point State::toGeoPoint() const {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}

geometry_msgs::Point State::toGlobalPoint(const double robot_yaw, const geometry_msgs::Point& robot_pos){
    Eigen::Matrix3d rot;
    rot.row(0) << std::cos(robot_yaw), -std::sin(robot_yaw), 0;
    rot.row(1) << std::sin(robot_yaw), std::cos(robot_yaw), 0;
    rot.row(2) << 0, 0, 1;

    Eigen::Vector3d p;
    p(0) = x;
    p(1) = y;
    p(2) = z;
    p = rot * p;

    geometry_msgs::Point pt;
    pt.x = p(0) + robot_pos.x;
    pt.y = p(1) + robot_pos.y;
    pt.z = p(2) + robot_pos.z;

    return pt;
}

geometry_msgs::Point State::toGlobalPoint(const double robot_yaw, const geometry_msgs::Point& robot_pos) const {
    Eigen::Matrix3d rot;
    rot.row(0) << std::cos(robot_yaw), -std::sin(robot_yaw), 0;
    rot.row(1) << std::sin(robot_yaw), std::cos(robot_yaw), 0;
    rot.row(2) << 0, 0, 1;

    Eigen::Vector3d p;
    p(0) = x;
    p(1) = y;
    p(2) = z;
    p = rot * p;

    geometry_msgs::Point pt;
    pt.x = p(0) + robot_pos.x;
    pt.y = p(1) + robot_pos.y;
    pt.z = p(2) + robot_pos.z;

    return pt;
}

PathStamp State::toStamp(){
    PathStamp stamp;
    stamp.x = x;
    stamp.y = y;
    stamp.z = z;
    stamp.yaw = yaw;
    stamp.gear = 1.0;
    return stamp;
}

PathStamp State::toStamp() const {
    PathStamp stamp;
    stamp.x = x;
    stamp.y = y;
    stamp.z = z;
    stamp.yaw = yaw;
    stamp.gear = 1.0;
    return stamp;
}

State State::GetLocalState(){
    State localState(0, 0, v, 0);
    return localState;
}

State State::operator+(const double& num) const{
    State temp(x + num, y + num, v + num, yaw + num);
    return temp;
}

State State::operator-(const double& num) const{
    State temp(x - num, y - num, v - num, yaw - num);
    return temp;
}

State State::operator*(const double& num) const{
    State temp(x * num, y * num, v * num, yaw * num);
    return temp;
}

State State::operator/(const double& num) const{
    State temp(x / num, y / num, v / num, yaw / num);
    return temp;
}

State State::operator+(const State& state) const{
    State temp(x + state.x, y + state.y, v + state.v, yaw + state.yaw);
    return temp;
}

State State::operator-(const State& state) const{
    State temp(x - state.x, y - state.y, v - state.v, yaw - state.yaw);
    return temp;
}

State State::operator*(const State& state) const{
    State temp(x * state.x, y * state.y, v * state.v, yaw * state.yaw);
    return temp;
}

State State::operator/(const State& state) const{
    State temp(x / state.x, y / state.y, v / state.v, yaw / state.yaw);
    return temp;
}

State& State::operator=(const double& num){
    x = num;
    y = num;
    v = num;
    yaw = num;
    return *this;
}

State& State::operator=(const State& state){
    x = state.x;
    y = state.y;
    z = state.z;
    yaw = state.yaw;
    v = state.v;
    w = state.w;
    orient = state.orient;
    return *this;
}

State& State::operator+=(const double& num){
    (*this) = (*this) + num;
    return *this;
}

State& State::operator-=(const double& num){
    (*this) = (*this) - num;
    return *this;
}

State& State::operator*=(const double& num){
    (*this) = (*this) * num;
    return *this;
}

State& State::operator/=(const double& num){
    (*this) = (*this) / num;
    return *this;
}

State& State::operator+=(const State& state){
    (*this) = (*this) + state;
    return *this;
}

State& State::operator-=(const State& state){
    (*this) = (*this) - state;
    return *this;
}

State& State::operator*=(const State& state){
    (*this) = (*this) * state;
    return *this;
}

State& State::operator/=(const State& state){
    (*this) = (*this) / state;
    return *this;
}

std::ostream& operator<<(std::ostream& os, const State& state){
    std::cout << std::setw(20) << "\t(x,y,z)" << ": (" << state.x << ", " << state.y << ", " << state.z << ")" << std::endl;
    std::cout << std::setw(20) << "\t(yaw,v,w)" << ": (" << state.yaw << ", " << state.v << ", " << state.w << ")" << std::endl;
    std::cout << std::setw(20) << "\t(q)" << ": (" << state.orient.x << ", " << state.orient.y << ", " << state.orient.z << ", " << state.orient.w << ")" << std::endl;
    return os;
}

