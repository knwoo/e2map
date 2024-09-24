#include "regression.h"


void Regressor::Fit(const std::vector<PathStamp>& ref){
    auto xys = Regressor::ToEigenVector(ref);
    auto xs = std::get<0>(xys);
    auto ys = std::get<1>(xys);
    auto A = Regressor::_GetVandermondeMat(xs);
    Regressor::SetCoeffs(A, ys);
}

double Regressor::GetFitValue(const double x){
    double value = 0.0;
    for (int i = 0; i < _coeffs.size(); i++) {
        value += _coeffs[i] * pow(x, i);
    }
    return value;
}

double Regressor::GetFunctionValue(const double x){
    double value = Regressor::GetFitValue(x);
    value = std::sin(path_yaw) * x + std::cos(path_yaw) * value + path_origin_y;
    return value;
}

double Regressor::GetTangentialAngle(const double x){
    double angle = 0;
    for (int i = 1; i < _coeffs.size(); i++) {
        angle += i * _coeffs[i] * std::pow(x, i-1);
    }
    angle = std::atan2(angle, 1);
    angle += path_yaw;
    return angle;
}

void Regressor::SetCoeffs(const Eigen::MatrixXd& A, const Eigen::VectorXd& ys){
    auto Q = A.householderQr();
    _coeffs = Q.solve(ys);

    if (_verbose){
        std::cout << prColor::Purple << "Poly Info : " << prColor::End << std::endl;
        for (int i=0; i < _coeffs.size(); i++){
            std::cout << prColor::Purple << std::setw(20) << "\tCoeff" << i << " : " << _coeffs[i] << prColor::End << std::endl;
        }
    }
}

bool Regressor::CheckSolved(){
    for (int i = 0; i < _coeffs.size(); i++){
        auto e = _coeffs(i);
        if (isnan(e) || isinf(e)){
            return false;
        }
    }
    return true;
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> Regressor::ToEigenVector(const std::vector<PathStamp>& ref){
    path_yaw = std::atan2(ref.at(1).y - ref.at(0).y, ref.at(1).x - ref.at(0).x);
    path_origin_x = ref.at(0).x;
    path_origin_y = ref.at(0).y;

    if (ref.at(0).gear < 0){
        path_yaw = Pi2Pi(path_yaw - M_PI);
    }
    Eigen::VectorXd xs(ref.size() + _window);
    Eigen::VectorXd ys(ref.size() + _window);
    for (int j=0; j < _window; j++){
        xs(j) = - ref.at(0).gear * _dl * (_window - j);
        ys(j) = 0;
    }
    for (int i=0; i < ref.size(); i++){
        double x = ref.at(i).x - path_origin_x;
        double y = ref.at(i).y - path_origin_y;
        xs(_window + i) = std::cos(-path_yaw) * x - std::sin(-path_yaw) * y;
        ys(_window + i) = std::sin(-path_yaw) * x + std::cos(-path_yaw) * y;
    }
    return std::make_tuple(xs, ys);
}

Eigen::MatrixXd Regressor::_GetVandermondeMat(const Eigen::VectorXd& xs){
    assert(_order >= 1 && _order <= xs.size() - 1);
    Eigen::MatrixXd A(xs.size(), _order + 1);

    for (int i = 0; i < xs.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xs.size(); j++) {
        for (int i = 0; i < _order; i++) {
            A(j, i + 1) = A(j, i) * xs(j);
        }
    }
    return A;
}

void Regressor::PrintConfig(){
    std::cout << "Poly Regressor : " << std::endl;
    std::cout << "\tOrder : " << _order << std::endl;
}

std::vector<PathStamp> Regressor::Getline(const std::vector<PathStamp>& ref){
    std::vector<PathStamp> line;
    for (const auto& stamp : ref){
        PathStamp fit_stamp;
        double value = Regressor::GetFitValue(stamp.x);
        fit_stamp.x = std::cos(path_yaw) * stamp.x - std::sin(path_yaw) * value + path_origin_x;
        fit_stamp.y = std::sin(path_yaw) * stamp.x + std::cos(path_yaw) * value + path_origin_y;
        fit_stamp.yaw = Regressor::GetTangentialAngle(stamp.x);
        line.push_back(fit_stamp);
    }
    return line;
}

int Regressor::GetOrder(){
    return _order;
}
int Regressor::GetVerbose(){
    return _verbose;
}

void Regressor::SetVerbose(bool verbose){
    _verbose = verbose;
}

bool Regressor::CheckDOF(double size){
    return size > _order ? true : false;
}

SplineRegressor::SplineRegressor(int num_knots, int order, int window, double dl, bool verbose)
:Regressor(order, window, dl, verbose), _num_knots(num_knots) {}

double SplineRegressor::_Hinge(double x, int p, double knot){
    if (x > knot){
        return std::pow((x - knot), p);
    } else {
        return 0;
    }
}
double SplineRegressor::_Poly(double x, int p){
    return std::pow(x, p);
}

void SplineRegressor::_SetBasis(){
    _basis.clear();
    for (int i = 0; i < Regressor::GetOrder() + 1; i++){
        auto poly_fn = [=](const double& x) -> double {
            return SplineRegressor::_Poly(x, i);
        };
        _basis.push_back(poly_fn);
    }
    for (const auto& knot : _knots){
        auto hinge_fn = [=](const double& x) -> double {
            return SplineRegressor::_Hinge(x, Regressor::GetOrder(), knot);
        };
        _basis.push_back(hinge_fn);
    }
}

void SplineRegressor::_SetKnots(const Eigen::VectorXd& xs){
    std::vector<double> vec;
    vec.resize(xs.size());
    Eigen::VectorXd::Map(&vec[0], xs.size()) = xs;
    std::sort(vec.begin(), vec.end());
    vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    double start = vec[0];
    double end = vec[vec.size() - 1];
    double segement = (end - start) / (double)(_num_knots + 1);
    _knots.clear();
    for (int i=0; i < _num_knots; i++){
        _knots.push_back(start + (i + 1) * segement);
    }
}

Eigen::MatrixXd SplineRegressor::_GetDataMat(const Eigen::VectorXd& xs){
    Eigen::MatrixXd mat(xs.size(), _basis.size());
    for (int i = 0; i < xs.size(); i++) {
        int j = 0;
        for (const auto& base_fn : _basis){
            mat(i, j) = base_fn(xs(i));
            j++;
        }
    }
    return mat;
}

void SplineRegressor::Fit(const std::vector<PathStamp>& ref){
    auto xys = Regressor::ToEigenVector(ref);
    auto xs = std::get<0>(xys);
    auto ys = std::get<1>(xys);

    SplineRegressor::_SetKnots(xs);
    SplineRegressor::_SetBasis();
    auto A = SplineRegressor::_GetDataMat(xs);
    SplineRegressor::SetCoeffs(A, ys);
}

double SplineRegressor::GetFitValue(const double x){
    double value = 0.0;
    int i = 0;
    for (const auto& base_fn : _basis){
        value += _coeffs[i] * base_fn(x);
        i++;
    }
    return value;
}

double SplineRegressor::GetFunctionValue(const double x){
    double value = SplineRegressor::GetFitValue(x);
    value = std::sin(path_yaw) * x + std::cos(path_yaw) * value + path_origin_y;
    return value;
}

double SplineRegressor::GetTangentialAngle(const double x){
    double angle = 0;
    for (int i = 1; i < Regressor::GetOrder(); i++) {
        angle += i * _coeffs[i] * std::pow(x, i-1);
    }
    for (int j=0; j < _num_knots; j++){
        angle += _coeffs[Regressor::GetOrder() + 1 + j] * Regressor::GetOrder() * std::pow(x - _knots.at(j), Regressor::GetOrder() - 1);
    }
    angle = std::atan2(angle, 1);
    angle += path_yaw;
    return angle;
}

void SplineRegressor::PrintConfig(){
    std::cout << "Spline Regressor : " << std::endl;
    std::cout << "\tOrder : " <<  Regressor::GetOrder() << std::endl;
    std::cout << "\tNumNnots : " << _num_knots << std::endl;
}

void SplineRegressor::SetCoeffs(const Eigen::MatrixXd& A, const Eigen::VectorXd& ys){
    auto Q = A.householderQr();
    _coeffs = Q.solve(ys);

    if (Regressor::GetVerbose()){
        std::cout << prColor::Purple << "Spline Info : " << prColor::End << std::endl;
        for (int i=0; i < _coeffs.size(); i++){
            std::cout << std::fixed << std::setprecision(2);
            std::cout << prColor::Purple << std::setw(20) << "\tCoeff" << i << " : " << _coeffs[i] << prColor::End << std::endl;
        }
    }
}

std::vector<PathStamp> SplineRegressor::Getline(const std::vector<PathStamp>& ref){
    std::vector<PathStamp> line;
    for (const auto& stamp : ref){
        PathStamp fit_stamp;
        double value = SplineRegressor::GetFitValue(stamp.x);
        fit_stamp.x = std::cos(path_yaw) * stamp.x - std::sin(path_yaw) * value + path_origin_x;
        fit_stamp.y = std::sin(path_yaw) * stamp.x + std::cos(path_yaw) * value + path_origin_y;
        fit_stamp.yaw = SplineRegressor::GetTangentialAngle(stamp.x);
        line.push_back(fit_stamp);
    }
    return line;
}

bool SplineRegressor::CheckDOF(double size){
    if (size > Regressor::GetOrder() + _num_knots) return true;
    return false;
}

int SplineRegressor::GetNumNnots(){
    return _num_knots;
}

SmoothingSplineRegressor::SmoothingSplineRegressor(double lamb, int num_knots, int order, int window, double dl, bool verbose)
:SplineRegressor(num_knots, order, window, dl, verbose), _lamb(lamb) {}

void SmoothingSplineRegressor::Fit(const std::vector<PathStamp>& ref){
    auto xys = Regressor::ToEigenVector(ref);
    auto xs = std::get<0>(xys);
    auto ys = std::get<1>(xys);

    SplineRegressor::_SetKnots(xs);
    SmoothingSplineRegressor::_SetBasis();
    auto N = SplineRegressor::_GetDataMat(xs);
    auto Omega = SmoothingSplineRegressor::_GetPenaltyMat(2);
    SmoothingSplineRegressor::SetCoeffs(N, Omega, ys);
}

double SmoothingSplineRegressor::_TruncPower(double x, int p){
    return (x > 0) ? std::pow(x, 2) : 0;
}

double SmoothingSplineRegressor::_TPD(double x, double knot, double max_knot){
    double numerator = SmoothingSplineRegressor::_TruncPower(x - knot) - SmoothingSplineRegressor::_TruncPower(x - max_knot);
    double denominator = max_knot - knot;
    return numerator / denominator;
}

double SmoothingSplineRegressor::_ResidualTPD(double x, double knot, double sub_knot, double max_knot){
    return SmoothingSplineRegressor::_TPD(x, knot, max_knot) - SmoothingSplineRegressor::_TPD(x, sub_knot, max_knot);
}

void SmoothingSplineRegressor::_SetBasis(){
    _basis.clear();
    auto base_fn1 = [=](const double& x) -> double { return 1;};
    _basis.push_back(base_fn1);
    auto base_fn2 = [=](const double& x) -> double { return x;};
    _basis.push_back(base_fn2);
    auto max_knot = _knots.at(_knots.size() - 1);
    auto sub_knot = _knots.at(_knots.size() - 2);
    for (int i = 0; i < _knots.size() - 2; i++){
        auto base_fn = [=](const double& x) -> double {
            return SmoothingSplineRegressor::_ResidualTPD(x, _knots.at(i), sub_knot, max_knot);
        };
        _basis.push_back(base_fn);
    }
}

Eigen::MatrixXd SmoothingSplineRegressor::_GetPenaltyMat(int target_idx){
    Eigen::MatrixXd Omega(_basis.size(), _basis.size());
    Omega.setZero();
    double min_knot = _knots.at(0);
    double max_knot =  _knots.at(_knots.size() - 1);
    std::vector<std::tuple<int, int, double>> results;
    for (int i = target_idx; i < _basis.size(); i++){
        for (int j=i; j < _basis.size(); j++){
            auto integrand_fn = [=](const double& x) -> double {
                return SmoothingSplineRegressor::_ProdSecondDeriv(x, _basis.at(i), _basis.at(j));
            };
            auto integration = SmoothingSplineRegressor::_Quad(integrand_fn, min_knot, max_knot);
            results.push_back(std::make_tuple(i, j, integration));
        }
    }

    for (const auto& result : results){
        int i = std::get<0>(result);
        int j = std::get<1>(result);
        double value = std::get<2>(result);
        Omega(i, j) = value;
    }

    Eigen::MatrixXd DiagOmega(_basis.size(), _basis.size());
    for (int i = 0; i < _basis.size(); i++){
        DiagOmega(i, i) = Omega(i, i);
    }
    Omega = Omega + Omega.transpose() - DiagOmega;
    return Omega;
}

double SmoothingSplineRegressor::_ProdSecondDeriv(double x, std::function<double(const double&)> func1, std::function<double(const double&)> func2, double dx){
    return SmoothingSplineRegressor::_Derivative(func1, x, dx, 2)*SmoothingSplineRegressor::_Derivative(func2, x, dx, 2);
}

double SmoothingSplineRegressor::_Derivative(std::function<double(const double&)> func, double x0, double dx, double n, int order){
    std::vector<double> weights;
    if (n == 1){
        weights = {-0.5, 0, 0.5};
    } else if (n == 2){
        weights = {1, -2.0, 1};
    } else{
        throw "myFunction is not implemented yet";
    }

    double value = 0.0;
    int ho = order >> 1;
    for (int k=0; k < order; k++){
        value += weights.at(k) * func(x0 + (k - ho) * dx);
    }
    std::vector<double> denom(n, dx);
    for (const auto e : denom){
        value /= e;
    }
    return value;
}

double SmoothingSplineRegressor::_Quad(std::function<double(const double&)> func, double lower, double upper, int sub_interval){
    double step_size = (upper - lower)/ sub_interval;
    double integration = func(lower) + func(upper);
    for(int i=1; i<= sub_interval-1; i++)
    {
        double k = lower + i * step_size;
        if(i % 3==0){
            integration = integration + 2 * (func(k));
        } else{
            integration = integration + 3 * (func(k));
        }
    }
    return integration = integration * step_size * 3.0 / 8.0;
}

void SmoothingSplineRegressor::SetCoeffs(const Eigen::MatrixXd& N, const Eigen::MatrixXd& Omega, const Eigen::VectorXd& ys){
    Eigen::MatrixXd A = N.transpose() * N + _lamb * Omega;

    auto Q = A.householderQr();
    _coeffs = Q.solve(N.transpose() * ys);
    _dof = (N * Q.solve(N.transpose())).trace();

    if (Regressor::GetVerbose()){
        double det = A.determinant();
        std::cout << prColor::Purple << "Smoothing Spline Info : " << prColor::End << std::endl;
        std::cout << prColor::Purple << std::setw(20) << "\tDOF" << " : " << _dof << prColor::End <<  std::endl;
        std::cout << prColor::Purple << std::setw(20) << "\tDet(A)" << " : " << det << prColor::End <<  std::endl;
        for (int i=0; i < _coeffs.size(); i++){
            std::cout << prColor::Purple << std::setw(20) << "\tCoeff" << i << " : " << _coeffs[i] << prColor::End << std::endl;
        }
    }
}

void SmoothingSplineRegressor::PrintConfig(){
    std::cout << "Smoothing Spline Regressor : " << std::endl;
    std::cout << "\tOrder : " <<  Regressor::GetOrder() << std::endl;
    std::cout << "\tLamb : " << _lamb << std::endl;
}

bool SmoothingSplineRegressor::CheckDOF(double size){
    if (size > _dof) return true;
    if (isnan(_dof) || isinf(_dof)) _dof = 1.0;
    if (Regressor::GetVerbose()) std::cout << prColor::Purple << std::setw(20) << "\tDOF" << " : " << _dof << prColor::End <<  std::endl;
    return false;
}

std::vector<PathStamp> SmoothingSplineRegressor::Getline(const std::vector<PathStamp>& ref){
    std::vector<PathStamp> line;
    for (const auto& stamp : ref){
        PathStamp fit_stamp;
        double value = SplineRegressor::GetFitValue(stamp.x);
        fit_stamp.x = std::cos(path_yaw) * stamp.x - std::sin(path_yaw) * value + path_origin_x;
        fit_stamp.y = std::sin(path_yaw) * stamp.x + std::cos(path_yaw) * value + path_origin_y;
        fit_stamp.yaw = SmoothingSplineRegressor::GetTangentialAngle(stamp.x);
        line.push_back(fit_stamp);
    }
    return line;
}

double SmoothingSplineRegressor::GetTangentialAngle(const double x){
    auto func = [=](const double& x) -> double {
        return SplineRegressor::GetFitValue(x);
    };
    double angle = SmoothingSplineRegressor::_Derivative(func, x);
    angle = std::atan2(angle, 1);
    angle += path_yaw;
    return angle;
}

void SmoothingSplineRegressor::Reset(){
    _dof = 1.0;
}

double PenalizedBSplineRegressor::_BSpline(double x, int order, int ind, const std::vector<double>& aug_knots){
    double c1 = 0;
    double c2 = 0;
    if (order == 0)
        return ((x >= aug_knots.at(ind)) && (x < aug_knots.at(ind + 1))) ? 1.0 : 0.0;
    if (aug_knots.at(ind + order) == aug_knots.at(ind)){
        c1 = 0.0;
    } else{
        c1 = (x - aug_knots.at(ind)) / (aug_knots.at(ind+order) - aug_knots.at(ind)) * PenalizedBSplineRegressor::_BSpline(x, order - 1, ind, aug_knots);
    }

    if (aug_knots.at(ind + order + 1) == aug_knots.at(ind + 1)){
        c2 = 0.0;
    } else{
        c2 = (aug_knots.at(ind + order + 1) - x) / (aug_knots.at(ind + order + 1) - aug_knots.at(ind + 1)) * PenalizedBSplineRegressor::_BSpline(x, order - 1, ind + 1, aug_knots);
    }
    return c1 + c2;
}

void PenalizedBSplineRegressor::_SetBasis(double eps){
    auto min_knot = _knots.at(0);
    auto max_knot = _knots.at(_knots.size() - 1);
    _knots.at(_knots.size() - 1) = max_knot + eps;
    auto order = Regressor::GetOrder();

    std::vector<double> aug_knots;
    for (int i = 0; i < order; i++){
        aug_knots.push_back(min_knot);
    }
    for (int i = 0; i < _knots.size(); i++){
        aug_knots.push_back(_knots.at(i));
    }
    for (int i = 0; i < order; i++){
        aug_knots.push_back(max_knot);
    }
    _basis.clear();
    for (int i=0; i < (order + _knots.size() - 1); i++){
        auto base_fn = [=](const double& x) -> double {
            return PenalizedBSplineRegressor::_BSpline(x, order, i, aug_knots);
        };
        _basis.push_back(base_fn);
    }
    _knots.resize(aug_knots.size());
    std::copy(aug_knots.begin(), aug_knots.end(), _knots.begin());
}

void PenalizedBSplineRegressor::Fit(const std::vector<PathStamp>& ref){
    auto xys = Regressor::ToEigenVector(ref);
    auto xs = std::get<0>(xys);
    auto ys = std::get<1>(xys);
    PenalizedBSplineRegressor::_SetKnots(xs);
    PenalizedBSplineRegressor::_SetBasis();
    auto N = SplineRegressor::_GetDataMat(xs);
    auto Omega = SmoothingSplineRegressor::_GetPenaltyMat(0);
    PenalizedBSplineRegressor::SetCoeffs(N, Omega, ys);
}

void PenalizedBSplineRegressor::PrintConfig(){
    std::cout << "Penalized BSpline Regressor : " << std::endl;
    std::cout << "\tOrder : " <<  Regressor::GetOrder() << std::endl;
    std::cout << "\tNumNnots : " << SplineRegressor::GetNumNnots() << std::endl;
}

void PenalizedBSplineRegressor::SetCoeffs(const Eigen::MatrixXd& N, const Eigen::MatrixXd& Omega, const Eigen::VectorXd& ys){
    Eigen::MatrixXd A = N.transpose() * N + _lamb * Omega;

    auto Q = A.householderQr();
    _coeffs = Q.solve(N.transpose() * ys);
    _dof = (N * Q.solve(N.transpose())).trace();

    if (Regressor::GetVerbose()){
        double det = A.determinant();
        std::cout << prColor::Purple << "Penalized BSpline Info : " << prColor::End << std::endl;
        std::cout << prColor::Purple << std::setw(20) << "\tDOF" << " : " << _dof << prColor::End <<  std::endl;
        std::cout << prColor::Purple << std::setw(20) << "\tDet(A)" << " : " << det << prColor::End <<  std::endl;
        std::cout << prColor::Purple << std::setw(20) << "\tKnots" << " : " << _knots << prColor::End <<  std::endl;
        for (int i=0; i < _coeffs.size(); i++){
            std::cout << prColor::Purple << std::setw(20) << "\tCoeff" << i << " : " << _coeffs[i] << prColor::End << std::endl;
        }
    }
}

void PenalizedBSplineRegressor::_SetKnots(const Eigen::VectorXd& xs){
    std::vector<double> vec;
    vec.resize(xs.size());
    Eigen::VectorXd::Map(&vec[0], xs.size()) = xs;
    std::sort(vec.begin(), vec.end());
    vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    double start = vec[0];
    double end = vec[vec.size() - 1];
    double segement = (end - start) / (double)(SplineRegressor::GetNumNnots() - 1);
    _knots.clear();
    for (int i=0; i < SplineRegressor::GetNumNnots(); i++){
        _knots.push_back(start + i * segement);
    }
}