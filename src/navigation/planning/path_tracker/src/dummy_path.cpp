#include "dummy_path.h"


std::vector<PathStamp> DummyPathGenerator::GetForwardStraightPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardStraightPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardTurnRightPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardTurnLeftPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardTurnRightPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardTurnLeftPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardInftyPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = 2 * M_PI / _curvature / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardInftyPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = 2 * M_PI / _curvature / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}


std::vector<PathStamp> DummyPathGenerator::GetForwardRandomPath(){
    std::vector<PathStamp> dummy_path;

    std::normal_distribution<double> dist(0.0, 1.0);
    double length = std::min(std::max(2.0 * dist(_gen) + M_PI, 1.0), 2 * M_PI);
    int n_step = length / _curvature / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;

}

std::vector<PathStamp> DummyPathGenerator::GetBackwardRandomPath(){
    std::vector<PathStamp> dummy_path;

    std::normal_distribution<double> dist(0.0, 1.0);
    double length = std::min(std::max(2.0 * dist(_gen) + M_PI, 1.0), 2 * M_PI);
    int n_step = length / _curvature / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardTrianglePath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardSquarePath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardPentagonPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardHourGlassPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }

    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI /3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI /3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardTrianglePath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardSquarePath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - M_PI / 2;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardPentagonPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 5;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardHourGlassPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw - 2 * M_PI / 3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }

    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI /3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.yaw = stamp.yaw + 2 * M_PI /3;
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetReturnStraightPath(){
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp;
    stamp.x = 0;
    stamp.y = 0;
    stamp.z = 0;
    stamp.yaw = 0;
    stamp.gear = 1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    stamp.gear = -1;
    dummy_path.push_back(stamp);
    for (int step=0; step < n_step; step++){
        stamp.x = stamp.x + stamp.gear * _dl * cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * sin(stamp.yaw);
        stamp.z = stamp.z;
        stamp.yaw = stamp.yaw;
        stamp.gear = stamp.gear;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::LoadCustomPath(const std::string& file_name){
    std::vector<PathStamp> dummy_path;

    std::string file_path = _pkg_loc + "/path/" + file_name;
    std::ifstream file(file_path);

    if (file.fail()){
        ROS_INFO("No File in path");
        std::cout << file_path << std::endl;
    } else {
        std::string line;
        int line_cnt = 0;
        while(std::getline(file, line)){
            if (line_cnt == 0){
                line_cnt ++;
                continue;
            }

            std::string cell;
            std::stringstream line_stream(line);
            std::vector<double> line_vec;
            while (std::getline(line_stream, cell, ','))
            {
                line_vec.push_back(std::stod(cell));
            }

            PathStamp stamp;
            stamp.x = line_vec.at(0);
            stamp.y = line_vec.at(1);
            stamp.z = line_vec.at(2);
            stamp.yaw = line_vec.at(3);
            stamp.gear = line_vec.at(4);
            dummy_path.push_back(stamp);
            line_cnt ++;
        }
    }
    return dummy_path;
}

void DummyPathGenerator::SaveCustomPath(const std::vector<PathStamp> path, const std::string& file_name){
    std::string dir_loc = _pkg_loc + "/path";

    std::string file_path = dir_loc + "/" + file_name;
    std::ofstream file;
    file.open(file_path);

    file << "x,y,z,yaw,gear\n";
    for (auto& stamp : path){
        std::string line = std::to_string(stamp.x) + "," + std::to_string(stamp.y) + "," + std::to_string(stamp.z) + "," + std::to_string(stamp.yaw) + "," + std::to_string(stamp.gear) + "\n";
        file << line;
    }
    file.close();
    ROS_INFO("Custom Path is saved");
    std::cout << file_path << std::endl;
}

std::tuple<std::vector<PathStamp>, State> DummyPathGenerator::ProcessCustomPath(Transform& trans, const std::vector<nav_msgs::Odometry>& history){
    std::vector<PathStamp> dummy_path;
    State base_state;
    for (int idx = 0; idx < history.size(); idx++){
        if (idx == 0){
            dummy_path.push_back(DummyPathGenerator::OdometryToStamp(trans, history.at(idx)));
            base_state = trans.GetState(history.at(idx).pose.pose.position, history.at(idx).pose.pose.orientation, history.at(idx).twist.twist);
            continue;
        }

        geometry_msgs::Vector3 res;
        res.x = history.at(idx).pose.pose.position.x - dummy_path.at((int)dummy_path.size() - 1).x;
        res.y = history.at(idx).pose.pose.position.y - dummy_path.at((int)dummy_path.size() - 1).y;
        res.z = 0;
        auto dist = GetNorm(res);
        if (dist >= _dl){
            dummy_path.push_back(DummyPathGenerator::OdometryToStamp(trans, history.at(idx)));
        }
    }
    return std::make_tuple(dummy_path, base_state);
}

PathStamp DummyPathGenerator::OdometryToStamp(Transform& trans, const nav_msgs::Odometry& odom){
    PathStamp stamp;
    auto state = trans.GetState(odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist);
    return state.toStamp();
}

void DummyPathGenerator::SetActive(bool active){
    _active = active;
    if (active == false){
        flag = false;
    }
}
void DummyPathGenerator::SetType(int type){
    _type = type;
}

void DummyPathGenerator::SetCurvature(double curvature){
    _curvature = curvature;
}

void DummyPathGenerator::SetLength(double length){
    _length = length;
}

bool DummyPathGenerator::GetActive(){
    return _active;
}
int DummyPathGenerator::GetType(){
    return _type;
}

double DummyPathGenerator::GetCurvature(){
    return _curvature;
}

double DummyPathGenerator::GetLength(){
    return _length;
}

std::string DummyPathGenerator::GetPkgPath(){
    return _pkg_loc;
}

void DummyPathGenerator::PrintConfig(){
    std::cout << "======================================================" << std::endl;
    std::cout << "Dummy Path Generator : " << std::endl;

    std::cout << "\tActive : " << _active << std::endl;
    std::cout << "\tType : " << _type << std::endl;
    std::cout << "\tCurvature : " << _curvature << std::endl;
    std::cout << "\tLength : " << _length << std::endl;
    std::cout << "======================================================" << std::endl;
}