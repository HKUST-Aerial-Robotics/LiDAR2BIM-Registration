/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3>
skewSymmetric(const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template<typename Derived>
Eigen::Quaternion<typename Derived::Scalar>
deltaQ(const Eigen::MatrixBase<Derived> &theta) {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    dq.normalize();
    return dq;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4>
Qleft(const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) =
                                                       qq.w() *
                                                       Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
                                                       skewSymmetric(qq.vec());
    return ans;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4>
Qright(const Eigen::QuaternionBase<Derived> &p) {
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) =
                                                       pp.w() *
                                                       Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
                                                       skewSymmetric(pp.vec());
    return ans;
}

template<typename T>
inline T R2Angle(const Eigen::Matrix<T, 3, 3> &R, bool is_degree = false) {
    T a = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
    if (a > 1) {
        a = 1;
    } else if (a < -1) {
        a = -1;
    }
    T angle = acos(a);
    if (is_degree) {
        angle = angle * 180.0 / M_PI;
    }
    return angle;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1>
R2ypr(const Eigen::MatrixBase<Derived> &R) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Matrix<Scalar_t, 3, 1> n = R.col(0);
    Eigen::Matrix<Scalar_t, 3, 1> o = R.col(1);
    Eigen::Matrix<Scalar_t, 3, 1> a = R.col(2);

    Eigen::Matrix<Scalar_t, 3, 1> ypr(3);
    Scalar_t y = atan2(n(1), n(0));
    Scalar_t p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    Scalar_t r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

template<typename T>
Eigen::Matrix<T, 3, 1>
R2ypr(const Eigen::Quaternion<T> &q) {
    // roll (x-axis rotation)
    T sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    T cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    T roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    T sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    T pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    T siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    T yaw = atan2(siny_cosp, cosy_cosp);
    return Eigen::Matrix<T, 3, 1>(yaw, pitch, roll);
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3>
ypr2R(const Eigen::MatrixBase<Derived> &ypr) {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

    return Rz * Ry * Rx;
}