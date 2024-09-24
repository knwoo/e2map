#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <ros/ros.h>

#include "state.h"


class Dynamics{
    public:
        Dynamics(){}
        ~Dynamics(){}

        void SetConfig(const ros::NodeHandle& nh);

        void PrintConfig();

        State UpdateState(const State& prevState, const Action& action);

        int GetNx();
        int GetNu();
        double GetT();
        double GetDT();
        double GetMaxLinVel();
        double GetMaxAngVel();

        void SetT(const int& T);
        void SetDT(const double& DT);

    private:
        int _NX;
        int _NU;
        int _T;
        double _DT;
        double _ICR;
        double _maxLinVel;
        double _maxAngVel;
};

#endif