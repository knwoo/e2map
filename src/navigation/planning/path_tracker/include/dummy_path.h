#ifndef DUMMY_PATH_H
#define DUMMY_PATH_H

#include <random>
#include <fstream>
// #include <filesystem>
#include <nav_msgs/Odometry.h>

#include "misc.h"


enum PathType{
    ForwardStraight = 0,
    BackwardStraight = 1,
    ForwardTurnRight = 2,
    ForwardTurnLeft = 3,
    BackwardTurnRight = 4,
    BackwardTurnLeft = 5,
    ForwardInfty = 6,
    BackwardInfty = 7,
    ForwardRandom = 8,
    BackwardRandom = 9,
    ForwardTriangle = 10,
    ForwardSquare = 11,
    ForwardPentagon = 12,
    ForwardHourGlass = 13,
    BackwardTriangle = 14,
    BackwardSquare = 15,
    BackwardPentagon = 16,
    BackwardHourGlass = 17,
    ReturnStraight = 18,
    SaveCustomPath = 19,
    CustomPath = 20
};

class DummyPathGenerator{
    public:
        DummyPathGenerator(std::string pkg_loc, double dl=0.1, bool active=false, double curvature=1.0, double length=5.0, int type=0)
        :_pkg_loc(pkg_loc), _dl(dl), _active(active), _curvature(curvature) ,_length(length), _type(type) {}

        DummyPathGenerator(){}

        ~DummyPathGenerator(){}

        std::vector<PathStamp> GetForwardStraightPath();
        std::vector<PathStamp> GetBackwardStraightPath();

        std::vector<PathStamp> GetForwardTurnRightPath();
        std::vector<PathStamp> GetForwardTurnLeftPath();
        std::vector<PathStamp> GetBackwardTurnRightPath();
        std::vector<PathStamp> GetBackwardTurnLeftPath();

        std::vector<PathStamp> GetForwardInftyPath();
        std::vector<PathStamp> GetBackwardInftyPath();

        std::vector<PathStamp> GetForwardRandomPath();
        std::vector<PathStamp> GetBackwardRandomPath();

        std::vector<PathStamp> GetForwardTrianglePath();
        std::vector<PathStamp> GetForwardSquarePath();
        std::vector<PathStamp> GetForwardPentagonPath();
        std::vector<PathStamp> GetForwardHourGlassPath();

        std::vector<PathStamp> GetBackwardTrianglePath();
        std::vector<PathStamp> GetBackwardSquarePath();
        std::vector<PathStamp> GetBackwardPentagonPath();
        std::vector<PathStamp> GetBackwardHourGlassPath();
        std::vector<PathStamp> GetReturnStraightPath();

        void SaveCustomPath(const std::vector<PathStamp> path, const std::string& file_name);
        std::vector<PathStamp> LoadCustomPath(const std::string& file_name);
        std::tuple<std::vector<PathStamp>, State> ProcessCustomPath(Transform& trans, const std::vector<nav_msgs::Odometry>& history);

        PathStamp OdometryToStamp(Transform& trans, const nav_msgs::Odometry& odom);

        void SetActive(bool active);
        void SetType(int type);
        void SetCurvature(double curvature);
        void SetLength(double length);

        bool GetActive();
        int GetType();
        double GetCurvature();
        double GetLength();

        std::string GetPkgPath();

        void PrintConfig();

        bool flag = false;

    private:
        std::string _pkg_loc;
        bool _active;
        double _dl;
        double _curvature;
        double _length;
        int _type;

        std::default_random_engine _gen;
};

#endif