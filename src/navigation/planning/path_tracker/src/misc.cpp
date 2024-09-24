#include "misc.h"


const char* prColor::Red = "\033[91m";
const char* prColor::Green = "\033[92m";
const char* prColor::Yellow = "\033[93m";
const char* prColor::LightPurple = "\033[94m";
const char* prColor::Purple = "\033[95m";
const char* prColor::Cyan = "\033[96m";
const char* prColor::LightGray = "\033[97m";
const char* prColor::End = "\033[00m";

double GetNorm(geometry_msgs::Vector3& vec){
    return std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2) + std::pow(vec.z, 2));
}

std::ostream& operator<<(std::ostream& os, const PathStamp& pt){
    os << "[" << pt.x << ", " << pt.y << ", " << pt.z<< "]";
    return os;
}

void CustomException::report(CustomException::ErrorType& e){
    switch (e){
        case CustomException::ErrorType::NotImplementedPath:
            ROS_INFO("%sNotImplementedPath Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::NotFoundTargetPath:
            ROS_INFO("%sNotFoundTargetPath Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::NotFoundReference:
            ROS_INFO("%sNotFoundReference Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::GoalArrived:
            ROS_INFO("%sGoalArrived%s", prColor::Green, prColor::End);
            break;
        case CustomException::ErrorType::NotFoundMapValue:
            ROS_INFO("%sNotFoundMapValue Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::NotFoundRegressorType:
            ROS_INFO("%sNotFoundRegressorType Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::NotInitalizeRegressor:
            ROS_INFO("%sNotInitalizeRegressor Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::NotSolvedRegressor:
            ROS_INFO("%sNotSolvedRegressorr Error%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::EmergencySituation:
            ROS_INFO("%sEmergencySituation%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::OutOdometry:
            ROS_INFO("%sOutOdometry%s", prColor::Red, prColor::End);
            break;
        case CustomException::ErrorType::OutTargetPath:
            ROS_INFO("%sOutTargetPath%s", prColor::Red, prColor::End);
            break;
    }
}

double GetMean(const std::vector<double>& vec){
    double sum = 0;
    for (const auto& e : vec){
        sum += e;
    }
    return sum / (double)vec.size();
}

double GetStd(const std::vector<double>& vec, const double mean){
    double std = 0;
    for (const auto& e : vec){
        std += std::pow(e - mean, 2);
    }
    return std::sqrt(std / (double)vec.size());
}

double GetStd(const std::vector<double>& vec){
    double mean = GetMean(vec);
    return GetStd(vec, mean);
}

double Pi2Pi(double angle){
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < - M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}