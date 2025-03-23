/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * author: QIAO Zhijian
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

template<typename T>
Eigen::Matrix<T, 3, 3> randomRotation(const T &angle_bound) {
    Eigen::Matrix<T, 3, 1> axis = Eigen::Matrix<T, 3, 1>::Random();
    axis.normalize();
    // random [-1, 1]
    T angle = angle_bound * Eigen::internal::random<T>() / 180.0 * M_PI;
    return Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
}

template<typename T>
Eigen::Matrix<T, 3, 1> randomTranslation(const T &translation_bound) {
    // random [-1, 1] for each element
    return Eigen::Matrix<T, 3, 1>::Random() * translation_bound;
}

template<typename T>
Eigen::Matrix<T, 4, 4> randomTransformation(const T &angle_bound, const T &translation_bound) {
    Eigen::Matrix<T, 4, 4> ans = Eigen::Matrix<T, 4, 4>::Identity();
    ans.template block<3, 3>(0, 0) = randomRotation(angle_bound);
    ans.template block<3, 1>(0, 3) = randomTranslation(translation_bound);
    return ans;
}

template<typename T>
inline T random_normal(T std) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<T> dis(0, std);
    T number = dis(gen);
    return number;
}

template<typename T>
inline T random_uniform(T min, T max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<T> dis(min, max);
    T number = dis(gen);
    return number;
}