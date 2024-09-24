#ifndef COST_H
#define COST_H

#include <vector>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>

#include "misc.h"
#include "state.h"
#include "action.h"

class Cost{
    public:

        bool map_arrived = false;
        std::vector<int> const_indices;

        Cost(){}
        ~Cost(){}

        void Reset();

        double GetShiftCost(const Action& u, const Action& v, const std::vector<double>& cov);
        double GetStateCost(const State& pred_state, const State& ref_state);
        double GetActionCost(const Action& action);
        double GetVelocityCost(const State& pred_state, double target_vel);
        double GetSubgoalCost(const State& pred_state, const PathStamp& subgoal);
        double GetHeadingCost(const geometry_msgs::Point& pred_state, const geometry_msgs::Point& subgoal, double robot_yaw);
        double GetTerminalCost(const State& pred_state, const State& ref_state);
        double GetMapCost(const geometry_msgs::Point& pt);
        bool GetMapConst(const geometry_msgs::Point& pt);

        void SetConfig(const ros::NodeHandle& nh);

        void PrintConfig();

        std::vector<double> GetQ();
        std::vector<double> GetQf();
        std::vector<double> GetR();
        std::vector<double> GetBound();
        double GetMapCostWeight();
        bool UseMapConst();

        void SetQ(std::vector<double> Q);
        void SetQf(std::vector<double> Qf);
        void SetR(std::vector<double> R);

        void SetCost(const nav_msgs::OccupancyGrid& cost_map);

    private:
        std::vector<double> _Q;
        std::vector<double> _Qf;
        std::vector<double> _R;
        double _cost_thres;
        double _subgoal_cost_weight;
        double _vel_cost_weight;
        double _heading_cost_weight;
        double _map_cost_weight;
        bool _map_const;

        double _origin_x, _origin_y, _map_x_size, _map_y_size;
        std::vector<double> _bounds;

        nav_msgs::OccupancyGrid _cost_map;

        bool _CheckOutsideBound(const geometry_msgs::Point& point);
        std::vector<int> _PointToGrid(const geometry_msgs::Point& point, const nav_msgs::MapMetaData& info);
        double _GetValue(const geometry_msgs::Point& point, const nav_msgs::OccupancyGrid& map);
};
#endif