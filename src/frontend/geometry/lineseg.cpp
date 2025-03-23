#include "frontend/geometry/lineseg.h"
#include <iostream>
#include <random>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>
#include <numeric>

LineSegment::LineSegment(const Eigen::Vector2d &pointA, const Eigen::Vector2d &pointB)
        : pointA_(pointA), pointB_(pointB) {
    initialize();
}

void LineSegment::initialize() {
    if (pointA_ == pointB_) {
        direction_ = Eigen::Vector2d::Random().normalized();
        length_ = 0;
    } else {
        direction_ = (pointB_ - pointA_).normalized();
        length_ = (pointB_ - pointA_).norm();
    }
    obs_.push_back(std::make_shared<LineSegment>(*this));
}

std::string LineSegment::toString() const {
    return "LineSegment: [" + std::to_string(pointA_[0]) + ", " + std::to_string(pointA_[1]) + "] -> [" +
           std::to_string(pointB_[0]) + ", " + std::to_string(pointB_[1]) + "]";
}

Eigen::Vector2d LineSegment::getPointA() const {
    return pointA_;
}

Eigen::Vector2d LineSegment::getPointB() const {
    return pointB_;
}

Eigen::Vector2d LineSegment::getDirection() const {
    return direction_;
}

void LineSegment::setPointA(const Eigen::Vector2d &pointA) {
    pointA_ = pointA;
    initialize();
}

void LineSegment::setPointB(const Eigen::Vector2d &pointB) {
    pointB_ = pointB;
    initialize();
}

std::vector<Eigen::Vector2d> LineSegment::samplePoints(double resolution) const {
    //	int num = static_cast<int>(length_ / resolution);
    //	std::vector<Eigen::Vector2d> points;
    //	for (int i = 0; i <= num; ++i) {
    //		points.push_back(pointA_ + i * resolution * direction_);
    //	}
    //	return points;
    // this is for keeping identical with the python code
    double length = (pointB_ - pointA_).norm();
    int num = static_cast<int>(length / resolution);
    std::vector<Eigen::Vector2d> points;
    if (num > 0) {
        for (int i = 0; i < num; ++i) {
            double t = static_cast<double>(i) / (num - 1);  // 计算插值因子
            points.push_back(pointA_ + t * (pointB_ - pointA_));
        }
    }
    return points;
}

LineSegment &LineSegment::random() {
    pointA_ = Eigen::Vector2d::Random();
    pointB_ = Eigen::Vector2d::Random();
    initialize();
    return *this;
}

LineSegment &LineSegment::transform(const Eigen::Matrix3d &rot, const Eigen::Vector2d &t) {
    Eigen::Vector3d pa = rot * Eigen::Vector3d(pointA_[0], pointA_[1], 1.0);
    Eigen::Vector3d pb = rot * Eigen::Vector3d(pointB_[0], pointB_[1], 1.0);
    pointA_ = pa.head<2>() + t;
    pointB_ = pb.head<2>() + t;
    direction_ = (pointB_ - pointA_).normalized();
    return *this;
}

LineSegment &LineSegment::fromCorners(const CornerPtr &cornerA, const CornerPtr &cornerB) {
    pointA_ = Eigen::Vector2d(cornerA->x_, cornerA->y_);
    pointB_ = Eigen::Vector2d(cornerB->x_, cornerB->y_);
    initialize();
    return *this;
}


Eigen::Vector2d LineSegment::getMidpoint() const {
    return (pointA_ + pointB_) / 2.0;
}

LineSegment &
LineSegment::fromMidpoint(const Eigen::Vector2d &midpoint, const Eigen::Vector2d &direction, double length) {
    pointA_ = midpoint - direction * length / 2.0;
    pointB_ = midpoint + direction * length / 2.0;
    direction_ = direction;
    length_ = length;
    return *this;
}

Eigen::Matrix2Xd LineSegment::getCorners() const {
    Eigen::Matrix2Xd points(2, 2);
    points.col(0) = pointA_;
    points.col(1) = pointB_;
    return points;
}

double LineSegment::getLength() const {
    return (pointA_ - pointB_).norm();
}

bool LineSegment::similar(const LineSegment &other, double angleThd, double ptThd) const {
    if ((distance(other.pointA_) < ptThd || distance(other.pointB_) < ptThd ||
         other.distance(pointA_) < ptThd || other.distance(pointB_) < ptThd)) {
        double cosSim = direction_.dot(other.direction_);
        if (cosSim > std::cos(angleThd * M_PI / 180.0)) {
            return true;
        }
    }
    return false;
}

double LineSegment::distance(const Eigen::Vector2d &point) const {
    if (pointA_ == pointB_) {
        return (point - pointA_).norm();
    }

    Eigen::Vector2d ap = point - pointA_;
    double projLength = ap.dot(direction_);
    Eigen::Vector2d closestPoint = pointA_ + projLength * direction_;

    if (projLength < 0) {
        closestPoint = pointA_;
    } else if (projLength > length_) {
        closestPoint = pointB_;
    }

    return (point - closestPoint).norm();
}

double LineSegment::angle(const LineSegment &other) const {
    return std::acos(std::abs(std::clamp(direction_.dot(other.direction_), -1.0, 1.0))) * 180.0 / M_PI;
}

void LineSegment::update(const LineSegment &other, bool avg) {
    if (avg) {
        Eigen::Vector2d newPointA = (pointA_ + other.pointA_) / 2.0;
        Eigen::Vector2d newPointB = (pointB_ + other.pointB_) / 2.0;
        reset(LineSegment(newPointA, newPointB));
    }
    obs_.push_back(std::make_shared<LineSegment>(other));
}

void LineSegment::reset(const LineSegment &other) {
    pointA_ = other.pointA_;
    pointB_ = other.pointB_;
    direction_ = other.direction_;
    length_ = other.length_;
}

LineSegments::LineSegments(const std::vector<LineSegment> &linesegments)
        : linesegments_(linesegments) {}

std::string LineSegments::toString() const {
    return "LineSegments: " + std::to_string(linesegments_.size());
}

std::vector<Eigen::Vector2d> LineSegments::samplePoints(double resolution) const {
    std::vector<Eigen::Vector2d> points;
    for (const auto &line: linesegments_) {
        auto sampled = line.samplePoints(resolution);
        points.insert(points.end(), sampled.begin(), sampled.end());
    }
    return points;
}

std::vector<Eigen::Vector2d> LineSegments::samplePoints3D(double resolution) const {
    std::vector<Eigen::Vector2d> points;
    for (const auto &line: linesegments_) {
        auto sampled = line.samplePoints(resolution);
        for (const auto &point: sampled) {
            points.push_back(Eigen::Vector2d(point.x(), point.y()));
        }
    }
    return points;
}

Eigen::MatrixXd LineSegments::samplePoints3dMatrix(double resolution) const {
    Eigen::MatrixXd points;
    for (const auto &line: linesegments_) {
        auto sampled = line.samplePoints(resolution);
        for (const auto &point: sampled) {
            points.conservativeResize(points.rows() + 1, 3);
            points.row(points.rows() - 1) << point.x(), point.y(), 0;
        }
    }
    return points;
}

void LineSegments::append(const LineSegment &line) {
    linesegments_.push_back(line);
}

void LineSegments::extend(double dist) {
    std::vector<LineSegment> newLinesegs;
    for (const auto &line: linesegments_) {
        LineSegment newLine = line;
        newLine.setPointA(newLine.getPointA() - dist * newLine.getDirection());
        newLine.setPointB(newLine.getPointB() + dist * newLine.getDirection());
        //newLine.setLength(newLine.getLength());
        newLinesegs.push_back(newLine);
    }
    linesegments_ = newLinesegs;
}

std::vector<CornerPtr> LineSegments::intersections(double threshold) const {
	std::vector<Corner> corners;
	for (size_t i = 0; i < linesegments_.size(); ++i) {
		for (size_t j = i + 1; j < linesegments_.size(); ++j) {
			const auto &line1 = linesegments_[i];
			const auto &line2 = linesegments_[j];
			if ((line1.getPointA() == line1.getPointB()) || (line2.getPointA() == line2.getPointB())) {
				continue;
			}
			auto x1 = line1.getPointA().x();
			auto y1 = line1.getPointA().y();
			auto x2 = line1.getPointB().x();
			auto y2 = line1.getPointB().y();
			auto x3 = line2.getPointA().x();
			auto y3 = line2.getPointA().y();
			auto x4 = line2.getPointB().x();
			auto y4 = line2.getPointB().y();
			
			double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
			if (denominator == 0) {
				continue;
			}
			double x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
			double y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;
			
			if ((std::min(x1, x2)-0.001 <= x && x <= std::max(x1, x2)+0.001) &&
				(std::min(x3, x4)-0.001 <= x && x <= std::max(x3, x4)+0.001) &&
				(std::min(y1, y2)-0.001 <= y && y <= std::max(y1, y2)+0.001) &&
				(std::min(y3, y4)-0.001 <= y && y <= std::max(y3, y4)+0.001)) {
				Corner corner(x, y);
				LineSegments intersectingLines({line1, line2});
				corner.setLines(intersectingLines);
				corners.push_back(corner);
			}
		}
	}
	
	// Remove duplicate corners within the threshold distance
	auto toRemove = std::vector<size_t>();
	for (size_t i = 0; i < corners.size(); ++i) {
		for (size_t j = i + 1; j < corners.size(); ++j) {
			if (corners[i].distance(corners[j]) < threshold) {
				toRemove.push_back(j);
			}
		}
	}
	std::sort(toRemove.begin(), toRemove.end(), std::greater<size_t>());
	toRemove.erase(std::unique(toRemove.begin(), toRemove.end()), toRemove.end());
	
	for (auto index: toRemove) {
		corners.erase(corners.begin() + index);
	}
	
	for (auto &corner: corners) {
		auto cornerXY = Eigen::Vector2d(corner.getXY().first, corner.getXY().second);
		std::vector<LineSegment> splitLines;
		for (const auto &line: *corner.getLines()) {
			splitLines.emplace_back(LineSegment(cornerXY, line.getPointA()));
			splitLines.emplace_back(LineSegment(cornerXY, line.getPointB()));
		}
		corner.setSplitLines(LineSegments(splitLines));
	}
	// Convert Corner to CornerPtr
	std::vector<CornerPtr> cornerPtrs;
	for (const auto &corner: corners) {
		cornerPtrs.push_back(std::make_shared<Corner>(corner));
	}
	return cornerPtrs;

}

LineSegments &LineSegments::transform(const Eigen::Matrix3d &rot, const Eigen::Vector2d &t) {
    for (auto &line: linesegments_) {
        line.transform(rot, t);
    }
    return *this;
}

Eigen::MatrixXi LineSegments::get2DNPArray() const {
    Eigen::MatrixXi lines(linesegments_.size(), 4);
    for (size_t i = 0; i < linesegments_.size(); ++i) {
        lines(i, 0) = static_cast<int>(linesegments_[i].getPointA().x());
        lines(i, 1) = static_cast<int>(linesegments_[i].getPointA().y());
        lines(i, 2) = static_cast<int>(linesegments_[i].getPointB().x());
        lines(i, 3) = static_cast<int>(linesegments_[i].getPointB().y());
    }
    return lines;
}

std::vector<Eigen::Matrix2d> LineSegments::get3DEigen() const {
    std::vector<Eigen::Matrix2d> lines;
    for (const auto &line: linesegments_) {
        Eigen::Matrix2d lineMat(2, 2);
        lineMat << line.getPointA().x(), line.getPointA().y(),
                line.getPointB().x(), line.getPointB().y();
        lines.push_back(lineMat);
    }
    return lines;

}

LineSegments &LineSegments::readFromFile(const std::string &filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double x1, y1, x2, y2;
        if (!(iss >> x1 >> y1 >> x2 >> y2)) {
            break;
        }
        linesegments_.emplace_back(Eigen::Vector2d(x1, y1), Eigen::Vector2d(x2, y2));
    }
    return *this;
}

void LineSegments::saveToFile(const std::string &filePath) const {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }
    for (const auto &line: linesegments_) {
        file << line.getPointA().x() << " " << line.getPointA().y() << " "
             << line.getPointB().x() << " " << line.getPointB().y() << "\n";
    }
}

std::vector<LineSegment>::iterator LineSegments::begin() {
    return linesegments_.begin();
}

std::vector<LineSegment>::iterator LineSegments::end() {
    return linesegments_.end();
}

std::vector<LineSegment>::const_iterator LineSegments::begin() const {
    return linesegments_.begin();
}

std::vector<LineSegment>::const_iterator LineSegments::end() const {
    return linesegments_.end();
}



