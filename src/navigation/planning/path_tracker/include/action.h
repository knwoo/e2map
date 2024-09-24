#ifndef ACTION_H
#define ACTION_H

#include <cmath>
#include "misc.h"


class Action{
    public:
        Action(){}
        Action(double vx, double wz)
        :vx(vx), wz(wz){
        }

        ~Action(){}

        std::vector<double> toVector();
        std::vector<double> toVector() const;

        double vx=0, wz=0;

        Action operator+(const double& num) const;
        Action operator-(const double& num) const;
        Action operator*(const double& num) const;
        Action operator/(const double& num) const;

        Action operator+(const Action& action) const;
        Action operator-(const Action& action) const;
        Action operator*(const Action& action) const;
        Action operator/(const Action& action) const;

        Action& operator=(const double& num);
        Action& operator=(const Action& action);

        Action& operator+=(const double& num);
        Action& operator-=(const double& num);
        Action& operator*=(const double& num);
        Action& operator/=(const double& num);

        Action& operator+=(const Action& action);
        Action& operator-=(const Action& action);
        Action& operator*=(const Action& action);
        Action& operator/=(const Action& action);

        friend std::ostream& operator<<(std::ostream& os, const Action& action);
};

#endif