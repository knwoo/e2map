#ifndef STATE_H
#define STATE_H

#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>

#include "misc.h"
#include "action.h"
#include "transform.h"


class PathStamp;

class State{
    public:
        State(){}

        State(double x, double y, double z, double yaw, double v, double w, geometry_msgs::Quaternion q)
        : x(x), y(y), z(z), yaw(yaw), v(v), w(w), orient(q){}

        State(double x, double y, double v, double yaw)
        : x(x), y(y), v(v), yaw(yaw){}

        State(std::vector<double> vecState)
        : x(vecState.at(0)), y(vecState.at(1)), v(vecState.at(2)), yaw(vecState.at(3)){}

        ~State(){}

        double x=0, y=0, z=0, yaw=0, v=0, w=0;
        geometry_msgs::Quaternion orient;

        double GetDistance(const State& state) const;

        double GetDistance(const geometry_msgs::Point& pt) const;

        double GetDistance(const PathStamp& stamp) const;

        double GetDistance(const double& p, const double& q, const double& r) const;

        std::vector<double> toVector();
        std::vector<double> toVector() const;

        geometry_msgs::Point toGeoPoint();
        geometry_msgs::Point toGeoPoint() const;

        geometry_msgs::Point toGlobalPoint(const double robot_yaw, const geometry_msgs::Point& robot_pos);
        geometry_msgs::Point toGlobalPoint(const double robot_yaw, const geometry_msgs::Point& robot_pos) const;

        PathStamp toStamp();
        PathStamp toStamp() const;

        State GetLocalState();

        State operator+(const double& num) const;
        State operator-(const double& num) const;
        State operator*(const double& num) const;
        State operator/(const double& num) const;

        State operator+(const State& state) const;
        State operator-(const State& state) const;
        State operator*(const State& state) const;
        State operator/(const State& state) const;

        State& operator=(const double& num);
        State& operator=(const State& state);

        State& operator+=(const double& num);
        State& operator-=(const double& num);
        State& operator*=(const double& num);
        State& operator/=(const double& num);

        State& operator+=(const State& state);
        State& operator-=(const State& state);
        State& operator*=(const State& state);
        State& operator/=(const State& state);


        friend std::ostream& operator<<(std::ostream& os, const State& state);


};

#endif