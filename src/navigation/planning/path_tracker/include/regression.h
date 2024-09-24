#ifndef REGRESSION_H
#define REGRESSION_H

#include <algorithm>
#include <Eigen/Core>
#include <Eigen/QR>

#include "misc.h"


class Regressor{
    public:
        Regressor(int order, int window, double dl, bool verbose)
        :_order(order), _window(window), _dl(dl), _verbose(verbose) {}

        Eigen::VectorXd _coeffs;
        double path_yaw;
        double path_origin_x;
        double path_origin_y;

        bool CheckSolved();
        int GetOrder();
        int GetVerbose();
        void SetVerbose(bool verbose);
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> ToEigenVector(const std::vector<PathStamp>& ref);

        virtual void Fit(const std::vector<PathStamp>& ref);
        virtual double GetFitValue(const double x);
        virtual double GetFunctionValue(const double x);
        virtual double GetTangentialAngle(const double x);
        virtual void SetCoeffs(const Eigen::MatrixXd& A, const Eigen::VectorXd& ys);
        virtual std::vector<PathStamp> Getline(const std::vector<PathStamp>& ref);
        virtual bool CheckDOF(double size);
        virtual void PrintConfig();
        virtual void Reset() {}

    private:
        int _order;
        int _window;
        double _dl;
        bool _verbose;

        Eigen::MatrixXd _GetVandermondeMat(const Eigen::VectorXd& xs);
};

class SplineRegressor: public Regressor {
    public:
        SplineRegressor(int num_knots, int order, int window, double dl, bool verbose);

        std::vector<double> _knots;
        std::vector<std::function<double(const double&)>> _basis;

        int GetNumNnots();
        virtual void Fit(const std::vector<PathStamp>& ref);
        virtual double GetFitValue(const double x);
        virtual double GetFunctionValue(const double x);
        virtual double GetTangentialAngle(const double x);
        virtual void SetCoeffs(const Eigen::MatrixXd& A, const Eigen::VectorXd& ys);
        virtual std::vector<PathStamp> Getline(const std::vector<PathStamp>& ref);
        virtual bool CheckDOF(double size);
        virtual void PrintConfig();
        virtual void Reset() {}
        virtual void _SetKnots(const Eigen::VectorXd& xs);

        Eigen::MatrixXd _GetDataMat(const Eigen::VectorXd& xs);

    private:
        int _num_knots;

        double _Hinge(double x, int p, double knot);
        double _Poly(double x, int p);
        void _SetBasis();
};

class SmoothingSplineRegressor: public SplineRegressor {
    public:
        SmoothingSplineRegressor(double lamb, int num_knots, int order, int window, double dl, bool verbose);

        double _lamb;
        double _dof = 1;

        virtual void Fit(const std::vector<PathStamp>& ref);
        virtual double GetTangentialAngle(const double x);
        virtual void SetCoeffs(const Eigen::MatrixXd& N, const Eigen::MatrixXd& Omega, const Eigen::VectorXd& ys);
        virtual std::vector<PathStamp> Getline(const std::vector<PathStamp>& ref);
        virtual bool CheckDOF(double size);
        virtual void PrintConfig();
        virtual void Reset();

        double _ProdSecondDeriv(double x, std::function<double(const double&)> func1, std::function<double(const double&)> func2, double dx = 1e-6);
        double _Derivative(std::function<double(const double&)> func, double x0, double dx = 1e-6, double n = 1, int order = 3);
        double _Quad(std::function<double(const double&)> func, double lower, double upper, int sub_interval = 12);
        Eigen::MatrixXd _GetPenaltyMat(int target_idx);

    private:
        double _TruncPower(double x, int p = 3);
        double _TPD(double x, double knot, double max_knot);
        double _ResidualTPD(double x, double knot, double sub_knot, double max_knot);
        void _SetBasis();
};

class PenalizedBSplineRegressor: public SmoothingSplineRegressor{
    public:
        PenalizedBSplineRegressor(double lamb, int num_knots, int order, int window, double dl, bool verbose)
        :SmoothingSplineRegressor(lamb, num_knots, order, window, dl, verbose) {}

        virtual void Fit(const std::vector<PathStamp>& ref);
        virtual void PrintConfig();
        virtual void SetCoeffs(const Eigen::MatrixXd& N, const Eigen::MatrixXd& Omega, const Eigen::VectorXd& ys);
        virtual void _SetKnots(const Eigen::VectorXd& xs);

    private:
        double _BSpline(double x, int order, int ind, const std::vector<double>& aug_knots);
        void _SetBasis(double eps=1e-6);
};
#endif