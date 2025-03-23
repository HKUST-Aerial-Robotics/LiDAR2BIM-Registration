#include "backend/hough/pose_2d.h"
Pose2D::Pose2D(double x, double y, double yaw)
		: x(x), y(y), yaw(yaw) {}

std::string Pose2D::toString() const {
	char buffer[50];
	std::sprintf(buffer, "Pose2D(x=%f, y=%f, yaw=%f)", x, y, yaw);
	return std::string(buffer);
}

bool Pose2D::operator==(const Pose2D& other) const {
	return x == other.x && y == other.y && yaw == other.yaw;
}

bool Pose2D::operator!=(const Pose2D& other) const {
	return !(*this == other);
}

Pose2D Pose2D::operator+(const Pose2D& other) const {
	return Pose2D(x + other.x, y + other.y, yaw + other.yaw);
}

Pose2D Pose2D::operator-(const Pose2D& other) const {
	return Pose2D(x - other.x, y - other.y, yaw - other.yaw);
}

Pose2D Pose2D::operator*(double scalar) const {
	return Pose2D(x * scalar, y * scalar, yaw * scalar);
}

Pose2D Pose2D::operator/(double scalar) const {
	if (scalar == 0) {
		throw std::invalid_argument("Division by zero");
	}
	return Pose2D(x / scalar, y / scalar, yaw / scalar);
}

Pose2D Pose2D::operator-() const {
	return Pose2D(-x, -y, -yaw);
}

Pose2D Pose2D::abs() const {
	return Pose2D(std::fabs(x), std::fabs(y), std::fabs(yaw));
}

void Pose2D::setSrcTriplets(const Eigen::Matrix<double, 3, 2>& srcTriplets) {
	this->srcTriplets_ = srcTriplets;
}

void Pose2D::setTriplets(const  Eigen::Matrix<double, 3, 2>& srcTriplets, const  Eigen::Matrix<double, 3, 2>& tgtTriplets) {
	this->srcTriplets_ = srcTriplets;
	this->tgtTriplets_ = tgtTriplets;
}

Polygon Pose2D::getSrcBoostTriplet() const {
	Polygon P;
	for (int i = 0; i < 3; ++i) {
		boost::geometry::append(P.outer(), Point(srcTriplets_(i, 0), srcTriplets_(i, 1)));
	}
	boost::geometry::append(P.outer(), Point(srcTriplets_(0, 0), srcTriplets_(0, 1))); // 闭合多边形
	if (!boost::geometry::is_valid(P)) {
		boost::geometry::correct(P);
	}
	return P;
}

//void Pose2D::getSrcCGALTriplet(Polygon_2& P, double random) const {
//	P.clear();  // 确保多边形为空
//	for (int i = 0; i < 3; ++i) {
//		P.push_back(Point_2(srcTriplets_(i, 0) + random, srcTriplets_(i, 1)+ random));
//	}
//}
