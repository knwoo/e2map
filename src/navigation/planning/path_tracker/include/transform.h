#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include "state.h"
#include "misc.h"

struct EulerAngle;
class State;


class Transform{
    public:
        geometry_msgs::Point ChangePointCoordinate(double x, double y, double z, Eigen::Matrix4d rot);
        geometry_msgs::Point ChangePointCoordinate(double x, double y, double z, Eigen::Matrix3d rot);

        std::vector<geometry_msgs::Point> TransformWithPosQuaternion(std::vector<geometry_msgs::Point> pts, geometry_msgs::Point p, geometry_msgs::Quaternion q);
        std::vector<geometry_msgs::Point> TransformWithPosEuler(std::vector<geometry_msgs::Point> pts, geometry_msgs::Point p, EulerAngle e);

        std::vector<geometry_msgs::Point> RotateWithQuaternion(std::vector<geometry_msgs::Point> pts, geometry_msgs::Quaternion q);
        std::vector<geometry_msgs::Point> RotateWithEuler(std::vector<geometry_msgs::Point> pts, EulerAngle e);

        EulerAngle GetEulerFromQuaternion(geometry_msgs::Quaternion q);
        geometry_msgs::Quaternion GetQuaternionFromEuler(EulerAngle e);

        EulerAngle GetEulerFromRotation(Eigen::Matrix3d rot);
        geometry_msgs::Quaternion GetQuaternionFromRotation(Eigen::Matrix3d rot);

        Eigen::Matrix3d GetRotationFromEuler(EulerAngle e);
        Eigen::Matrix4d GetRotationFromQuaternion(geometry_msgs::Quaternion q);

        geometry_msgs::Quaternion MultiplyQuaternion(geometry_msgs::Quaternion p, geometry_msgs::Quaternion q);

        geometry_msgs::Point GetPos();
        geometry_msgs::Quaternion GetOrient();
        geometry_msgs::Twist GetTwist();

        void SetPos(geometry_msgs::Point p);
        void SetOrient(geometry_msgs::Quaternion q);
        void SetTwist(geometry_msgs::Twist t);

        State GetState(geometry_msgs::Point p, geometry_msgs::Quaternion q, geometry_msgs::Twist t);

    private:
        geometry_msgs::Point _pos;
        geometry_msgs::Quaternion _orient;
        geometry_msgs::Twist _twist;

};

#endif