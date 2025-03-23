/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_CORNER_H
#define BIMREG_CORNER_H
#include <cmath>
#include <vector>
#include <memory>
#include <Eigen/Dense>
// Forward declaration of LineSegments class
class LineSegments;
class Corner {
public:
	Corner(double x, double y);
	Corner() = default;
	std::string toString() const;
	void setLines(const LineSegments& lines);
	void setSplitLines(const LineSegments& splitLines);
	double distance(const Corner& other) const;
	std::pair<double, double> getXY() const;
	std::shared_ptr<LineSegments> getLines() const;
	std::shared_ptr<LineSegments> getSplitLines() const;
	double x_;
	double y_;
private:

	std::shared_ptr<LineSegments> lines_;
	std::shared_ptr<LineSegments> splitLines_;
};

using CornerPtr = std::shared_ptr<Corner>;
using CornerVector = std::vector<CornerPtr>;
using CornerTriplets = std::vector<CornerVector>;
#endif //BIMREG_CORNER_H
