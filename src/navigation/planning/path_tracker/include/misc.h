#ifndef MISC_H
#define MISC_H

#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include <geometry_msgs/Point.h>

#include "state.h"


class CustomException : public std::exception
{
    public:
        enum ErrorType{
            NotImplementedPath,
            GoalArrived,
            NotFoundTargetPath,
            NotFoundReference,
            NotFoundMapValue,
            NotFoundRegressorType,
            NotInitalizeRegressor,
            NotSolvedRegressor,
            EmergencySituation,
            OutOdometry,
            OutTargetPath,
        };

        static void report(CustomException::ErrorType& e);
};

struct EulerAngle {
    double roll, pitch, yaw;
};

class PathStamp{
    public:
        double x;
        double y;
        double z;
        double yaw;
        double gear;

        friend std::ostream& operator<<(std::ostream& os, const PathStamp& pt);
};

namespace prColor {
    extern const char* Red;
    extern const char* Green;
    extern const char* Yellow;
    extern const char* LightPurple;
    extern const char* Purple;
    extern const char* Cyan;
    extern const char* LightGray;
    extern const char* End;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec){
    os.precision(3);
    os << "[";
    int cnt = 1;
    for(auto it = vec.begin(); it != vec.end(); it++){
        os << *it;
        if(it != vec.end() - 1){
            os << ", ";
        }
        if(cnt % 5 == 0 && cnt!=vec.size()){
            os << "\n";
        }
        cnt = cnt + 1;
    }
    os << "]";
    return os;
}

template <typename T>
std::vector<T> slicing(const std::vector<T>& arr, int begin_ind, int end_ind){
    end_ind = std::min(end_ind, (int)arr.size());

    auto start = arr.begin() + begin_ind;
    auto end = arr.begin() + end_ind;

    std::vector<T> result(end_ind - begin_ind);
    std::copy(start, end, result.begin());
    return result;
}

double GetNorm(geometry_msgs::Vector3& vec);

double GetMean(const std::vector<double>& vec);

double GetStd(const std::vector<double>& vec, const double mean);

double GetStd(const std::vector<double>& vec);

double Pi2Pi(double angle);

#endif