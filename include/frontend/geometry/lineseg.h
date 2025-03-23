/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_LINESEG_H
#define BIMREG_LINESEG_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <memory>
#include "frontend/geometry/corner.h"

class LineSegment {
public:
    LineSegment(const Eigen::Vector2d &pointA = Eigen::Vector2d::Zero(),
                const Eigen::Vector2d &pointB = Eigen::Vector2d::Zero());

    std::string toString() const;

    std::vector<Eigen::Vector2d> samplePoints(double resolution = 0.1) const;

    LineSegment &random();

    LineSegment &transform(const Eigen::Matrix3d &rot, const Eigen::Vector2d &t);

    LineSegment &fromCorners(const CornerPtr &cornerA, const CornerPtr &cornerB);

    Eigen::Vector2d getMidpoint() const;

    LineSegment &fromMidpoint(const Eigen::Vector2d &midpoint, const Eigen::Vector2d &direction, double length);

    Eigen::Matrix2Xd getCorners() const;

    double getLength() const;

    bool similar(const LineSegment &other, double angleThd = 5.0, double ptThd = 0.2) const;

    double distance(const Eigen::Vector2d &point) const;

    double angle(const LineSegment &other) const;

    void update(const LineSegment &other, bool avg = true);

    void reset(const LineSegment &other);

    Eigen::Vector2d getPointA() const;

    Eigen::Vector2d getPointB() const;

    Eigen::Vector2d getDirection() const;

    void setPointA(const Eigen::Vector2d &pointA);

    void setPointB(const Eigen::Vector2d &pointB);

private:
    Eigen::Vector2d pointA_;
    Eigen::Vector2d pointB_;
    Eigen::Vector2d direction_;
    double length_;
    std::vector<std::shared_ptr<LineSegment>> obs_;

    void initialize();
};

class LineSegments {
public:
    LineSegments(const std::vector<LineSegment> &linesegments = {});

    std::string toString() const;

    std::vector<Eigen::Vector2d> samplePoints(double resolution = 0.1) const;

    std::vector<Eigen::Vector2d> samplePoints3D(double resolution = 0.1) const;

    Eigen::MatrixXd samplePoints3dMatrix(double resolution = 0.1) const;

    void append(const LineSegment &line);

    void extend(double dist = 0.1);

    std::vector<CornerPtr> intersections(double threshold = 0.1) const;

    LineSegments &transform(const Eigen::Matrix3d &rot, const Eigen::Vector2d &t);

    Eigen::MatrixXi get2DNPArray() const;

    std::vector<Eigen::Matrix2d> get3DEigen() const;

    LineSegments &readFromFile(const std::string &filePath);

    void saveToFile(const std::string &filePath) const;

    // Iterator support
    std::vector<LineSegment>::iterator begin();

    std::vector<LineSegment>::iterator end();

    std::vector<LineSegment>::const_iterator begin() const;

    std::vector<LineSegment>::const_iterator end() const;

private:
    std::vector<LineSegment> linesegments_;
};

#endif //BIMREG_LINESEG_H
