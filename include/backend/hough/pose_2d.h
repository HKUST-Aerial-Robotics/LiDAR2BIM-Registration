/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#ifndef BIMREG_POSE_2D_H
#define BIMREG_POSE_2D_H

#include <vector>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <Eigen/Dense>

#include <list>

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::polygon<Point> Polygon;
typedef boost::geometry::model::multi_polygon<Polygon> MultiPolygon;

class Pose2D {
public:
    double x, y, yaw;

    Pose2D(double x = 0, double y = 0, double yaw = 0);

    std::string toString() const;

    bool operator==(const Pose2D &other) const;

    bool operator!=(const Pose2D &other) const;

    Pose2D operator+(const Pose2D &other) const;

    Pose2D operator-(const Pose2D &other) const;

    Pose2D operator*(double scalar) const;

    Pose2D operator/(double scalar) const;

    Pose2D operator-() const;

    Pose2D abs() const;

    void setSrcTriplets(const Eigen::Matrix<double, 3, 2> &srcTriplets);

    void setTriplets(const Eigen::Matrix<double, 3, 2> &srcTriplets, const Eigen::Matrix<double, 3, 2> &tgtTriplets);
	
	Eigen::Matrix<double, 3, 2> getSrcTriplets() const{
		return srcTriplets_;
	}
	Eigen::Matrix<double, 3, 2> getTgtTriplets() const{
		return tgtTriplets_;
	}

//	void getSrcCGALTriplet(Polygon_2& P, double random) const;
    Polygon getSrcBoostTriplet() const;

private:
    Eigen::Matrix<double, 3, 2> srcTriplets_;
    Eigen::Matrix<double, 3, 2> tgtTriplets_;
};


#endif //BIMREG_POSE_2D_H
