#include "frontend/geometry/corner.h"
#include "frontend/geometry/lineseg.h"

Corner::Corner(double x, double y) : x_(std::round(x * 100) / 100.0), y_(std::round(y * 100) / 100.0) {}

std::string Corner::toString() const {
	return std::to_string(x_) + " " + std::to_string(y_);
}

void Corner::setLines(const LineSegments& lines) {
	lines_ = std::make_shared<LineSegments>(lines);
}

void Corner::setSplitLines(const LineSegments& splitLines) {
	splitLines_ = std::make_shared<LineSegments>(splitLines);
}

double Corner::distance(const Corner& other) const {
	return std::sqrt(std::pow(x_ - other.x_, 2) + std::pow(y_ - other.y_, 2));
}

std::pair<double, double> Corner::getXY() const {
	return {x_, y_};
}

std::shared_ptr<LineSegments> Corner::getLines() const {
	return lines_;
}

std::shared_ptr<LineSegments> Corner::getSplitLines() const {
	return splitLines_;
}