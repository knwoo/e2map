#include "transform.h"

geometry_msgs::Point Transform::ChangePointCoordinate(double x, double y, double z, Eigen::Matrix4d rot){
    Eigen::Vector4d p;
    p(0) = x;
    p(1) = y;
    p(2) = z;
    p(3) = 1;
    p = rot * p;

    geometry_msgs::Point pt;
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    return pt;
}

geometry_msgs::Point Transform::ChangePointCoordinate(double x, double y, double z, Eigen::Matrix3d rot){
    Eigen::Vector3d p;
    p(0) = x;
    p(1) = y;
    p(2) = z;
    p = rot * p;

    geometry_msgs::Point pt;
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    return pt;
}

std::vector<geometry_msgs::Point> Transform::TransformWithPosQuaternion(std::vector<geometry_msgs::Point> pts, geometry_msgs::Point p, geometry_msgs::Quaternion q){
    auto rot = Transform::GetRotationFromQuaternion(q);
    rot.col(3) << p.x, p.y, p.z, 1;
    std::vector<geometry_msgs::Point> changedPts;
    std::transform(pts.begin(), pts.end(), std::back_inserter(changedPts), [&](geometry_msgs::Point pt) -> geometry_msgs::Point {
        return Transform::ChangePointCoordinate(pt.x, pt.y, pt.z, rot);
    });
    return changedPts;
}

std::vector<geometry_msgs::Point> Transform::TransformWithPosEuler(std::vector<geometry_msgs::Point> pts, geometry_msgs::Point p, EulerAngle e){
    auto _rot = Transform::GetRotationFromEuler(e);
    Eigen::Matrix4d rot;
    rot.col(0) << _rot(0,0), _rot(1,0), _rot(2,0), 0;
    rot.col(1) << _rot(0,1), _rot(1,1), _rot(2,1), 0;
    rot.col(2) << _rot(0,2), _rot(1,2), _rot(2,2), 0;
    rot.col(3) << p.x, p.y, p.z, 1;
    std::vector<geometry_msgs::Point> changedPts;
    std::transform(pts.begin(), pts.end(), std::back_inserter(changedPts), [&](geometry_msgs::Point pt) -> geometry_msgs::Point {
        return Transform::ChangePointCoordinate(pt.x, pt.y, pt.z, rot);
    });
    return changedPts;
}

std::vector<geometry_msgs::Point> Transform::RotateWithQuaternion(std::vector<geometry_msgs::Point> pts, geometry_msgs::Quaternion q){
    Eigen::Matrix4d rot = Transform::GetRotationFromQuaternion(q);
    std::vector<geometry_msgs::Point> changedPts;
    std::transform(pts.begin(), pts.end(), std::back_inserter(changedPts), [&](geometry_msgs::Point pt) -> geometry_msgs::Point {
        return Transform::ChangePointCoordinate(pt.x, pt.y, pt.z, rot);
    });
    return changedPts;
}

std::vector<geometry_msgs::Point> Transform::RotateWithEuler(std::vector<geometry_msgs::Point> pts, EulerAngle e){
    auto rot = Transform::GetRotationFromEuler(e);
    std::vector<geometry_msgs::Point> changedPts;
    std::transform(pts.begin(), pts.end(), std::back_inserter(changedPts), [&](geometry_msgs::Point pt) -> geometry_msgs::Point {
        return Transform::ChangePointCoordinate(pt.x, pt.y, pt.z, rot);
    });
    return changedPts;
}

EulerAngle Transform::GetEulerFromQuaternion(geometry_msgs::Quaternion q){
    EulerAngle e;

    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    e.roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    e.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    e.yaw = std::atan2(siny_cosp, cosy_cosp);

    return e;
}

geometry_msgs::Quaternion Transform::GetQuaternionFromEuler(EulerAngle e){
    double cr = cos(e.roll * 0.5);
    double sr = sin(e.roll * 0.5);
    double cp = cos(e.pitch * 0.5);
    double sp = sin(e.pitch * 0.5);
    double cy = cos(e.yaw * 0.5);
    double sy = sin(e.yaw * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

EulerAngle Transform::GetEulerFromRotation(Eigen::Matrix3d rot){
    auto q = Transform::GetQuaternionFromRotation(rot);
    auto e = Transform::GetEulerFromQuaternion(q);
    return e;
}

geometry_msgs::Quaternion Transform::GetQuaternionFromRotation(Eigen::Matrix3d rot){
    double m00 = rot(0,0);
    double m01 = rot(0,1);
    double m02 = rot(0,2);
    double m10 = rot(1,0);
    double m11 = rot(1,1);
    double m12 = rot(1,2);
    double m20 = rot(2,0);
    double m21 = rot(2,1);
    double m22 = rot(2,2);

    double tr = m00 + m11 + m22;
    geometry_msgs::Quaternion q;
    if (tr > 0){
        // S=4*qw
        double S = std::sqrt(tr + 1.0) * 2;
        q.w = 0.25 * S;
        q.x = (m21 - m12) / S;
        q.y = (m02 - m20) / S;
        q.z = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)){
        // S=4*qx
        double S = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        q.w = (m21 - m12) / S;
        q.x = 0.25 * S;
        q.y = (m01 + m10) / S;
        q.z = (m02 + m20) / S;
    } else if (m11 > m22) {
        // S=4*qy
        double S = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        q.w = (m02 - m20) / S;
        q.x = (m01 + m10) / S;
        q.y = 0.25 * S;
        q.z = (m12 + m21) / S;
    } else {
        // S=4*qz
        double S = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        q.w = (m10 - m01) / S;
        q.x = (m02 + m20) / S;
        q.y = (m12 + m21) / S;
        q.z = 0.25 * S;
    }
    return q;

}

Eigen::Matrix3d Transform::GetRotationFromEuler(EulerAngle e){
    Eigen::Matrix3d R_x;
    R_x.row(0) << 1, 0, 0;
    R_x.row(1) << 0, std::cos(e.roll), -std::sin(e.roll);
    R_x.row(2) << 0, std::sin(e.roll), std::cos(e.roll);

    Eigen::Matrix3d R_y;
    R_y.row(0) << std::cos(e.pitch), 0, std::sin(e.pitch);
    R_y.row(1) << 0, 1, 0;
    R_y.row(2) << -std::sin(e.pitch), 0, std::cos(e.pitch);

    Eigen::Matrix3d R_z;
    R_z.row(0) << std::cos(e.yaw), -std::sin(e.yaw), 0;
    R_z.row(1) << std::sin(e.yaw), std::cos(e.yaw), 0;
    R_z.row(2) << 0, 0, 1;

    Eigen::Matrix3d rotationMatrix = R_z * R_y * R_x;

    return rotationMatrix;
}

Eigen::Matrix4d Transform::GetRotationFromQuaternion(geometry_msgs::Quaternion q){
    Eigen::Matrix4d Rq;;
    Rq.row(0) << q.w, q.z, -q.y, q.x;
    Rq.row(1) << -q.z, q.w, q.x, q.y;
    Rq.row(2) << q.y, -q.x, q.w, q.z;
    Rq.row(3) << -q.x, -q.y, -q.z, q.w;

    Eigen::Matrix4d Lq;
    Lq.row(0) << q.w, q.z, -q.y, -q.x;
    Lq.row(1) << -q.z, q.w, q.x, -q.y;
    Lq.row(2) << q.y, -q.x, q.w, -q.z;
    Lq.row(3) << q.x, q.y, q.z, q.w;
    return Rq * Lq;
}

geometry_msgs::Quaternion Transform::MultiplyQuaternion(geometry_msgs::Quaternion p, geometry_msgs::Quaternion q){
    geometry_msgs::Quaternion r;
    r.x = p.x * q.x - p.y * q.y - p.z * q.z - p.w * q.w;
    r.y = p.x * q.y + p.y * q.x + p.z * q.w - p.w * q.z;
    r.z = p.x * q.z - p.y * q.w + p.z * q.x + p.w * q.y;
    r.w = p.x * q.w + p.y * q.z - p.z * q.y + p.w * q.x;
    return r;
}

geometry_msgs::Point Transform::GetPos(){
    return _pos;
}

geometry_msgs::Quaternion Transform::GetOrient(){
    return _orient;
}

geometry_msgs::Twist Transform::GetTwist(){
    return _twist;
}

void Transform::SetPos(geometry_msgs::Point p){
    _pos = p;
}

void Transform::SetOrient(geometry_msgs::Quaternion q){
    _orient = q;
}

void Transform::SetTwist(geometry_msgs::Twist t){
    _twist = t;
}

State Transform::GetState(geometry_msgs::Point p, geometry_msgs::Quaternion q, geometry_msgs::Twist t){
    auto e = Transform::GetEulerFromQuaternion(q);
    auto t_new = t;
    t_new.linear.z = 0;
    double v = t_new.linear.x;
    double w = t.angular.z;
    return State(p.x, p.y, p.z, e.yaw, v, w, q);
}