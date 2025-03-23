#include "utils/raster_utils.h"

PointRaster::PointRaster(const std::vector<Eigen::Vector2d> &points, double gridSize)
        : points_(points), gridSize_(gridSize) {
    this->initRaster(points);
}

PointRaster::PointRaster(double gridSize) : gridSize_(gridSize) {

}

void PointRaster::initRaster(const std::vector<Eigen::Vector2d> &points) {

    Eigen::Vector2d minPoint = points[0];
    Eigen::Vector2d maxPoint = points[0];
    for (const auto &point: points) {
        minPoint = minPoint.array().min(point.array());
        maxPoint = maxPoint.array().max(point.array());
    }

    origin_ = minPoint;

    int width = static_cast<int>(std::ceil((maxPoint[0] - minPoint[0]) / gridSize_)) + 2;
    int height = static_cast<int>(std::ceil((maxPoint[1] - minPoint[1]) / gridSize_)) + 2;
    size_ = Eigen::Vector2i(width, height);
}

void PointRaster::rasterize(const std::vector<Eigen::Vector2d> &points, double invert, bool override, int width) {
    const auto &pts = points.empty() ? this->points_ : points;
    auto grids = pointToGrid(pts, width);
    for (size_t i = 0; i < pts.size(); ++i) {
        const auto &p = pts[i];
        const auto &grid = grids[i];
        if (this->grids_.find(grid) == this->grids_.end()) {
            this->grids_[grid] = {p};
            this->gridsNum_[grid] = 1 * invert;
        } else {
            if (override) {
                this->grids_[grid].push_back(p);
                this->gridsNum_[grid] += 1 * invert;
            }
        }
    }
}
void PointRaster::rasterize_free(const std::vector<Eigen::Vector2d> &points, double invert, bool override, int width) {
    const auto &pts = points;
    auto grids = pointToGrid(pts, width);
    for (size_t i = 0; i < pts.size(); ++i) {
        const auto &p = pts[i];
        const auto &grid = grids[i];
        if (this->grids_.find(grid) == this->grids_.end()) {
            this->grids_[grid] = {p};
            this->gridsNum_[grid] = 1 * invert;
        } else {
            if (override) {
                this->grids_[grid].push_back(p);
                this->gridsNum_[grid] += 1 * invert;
            }
        }
    }
}

std::vector<Eigen::Vector2i> PointRaster::pointToGrid(const std::vector<Eigen::Vector2d> &points, int width) {
    std::vector<Eigen::Vector2i> grids;
    if (width == 0) {
        for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector2i grid = ((points[i] - origin_) / gridSize_).array().round().cast<int>();
            grids.push_back(grid);
        }
    } else {
        std::map<Eigen::Vector2i, int, compareVector2i> gridsDict;
        for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector2i grid = ((points[i] - origin_) / gridSize_).array().round().cast<int>();
            for (int dx = -width; dx <= width; ++dx) {
                for (int dy = -width; dy <= width; ++dy) {
                    Eigen::Vector2i key(grid[0] + dx, grid[1] + dy);
                    if (gridsDict.find(key) == gridsDict.end()) {
                        gridsDict[key] = 1;
                        grids.push_back(key);
                    }
                }
            }
        }
    }
    return grids;
}

Eigen::Vector2d PointRaster::gridToPoint(const Eigen::Vector2i &grid) {
    return grid.cast<double>() * gridSize_ + origin_;
}

double PointRaster::getObsNum(const Eigen::Vector2i &key, int width) {
    if (width == 0) {
        return gridsNum_.find(key) != gridsNum_.end() ? gridsNum_[key] : 0;
    } else {
        int num = 0;
        for (int dx = -width; dx <= width; ++dx) {
            for (int dy = -width; dy <= width; ++dy) {
                Eigen::Vector2i neighborKey(key[0] + dx, key[1] + dy);
                num += getObsNum(neighborKey);
            }
        }
        return num;
    }
}

std::vector<Eigen::Vector2d> PointRaster::getCenters(const std::vector<Eigen::Vector2i> &grids) {
    std::vector<Eigen::Vector2d> centers;
    std::vector<Eigen::Vector2i> grids_used = grids;
    if (grids.empty()) {
        for (const auto &pair: this->grids_) {
            if (!pair.second.empty()) {
                grids_used.push_back(pair.first);
            }
        }
    }
    for (const auto &grid: grids_used) {
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        for (const auto &point: grids_.at(grid)) {
            center += point;
        }
        center /= grids_.at(grid).size();
        centers.push_back(center);
    }
    return centers;
}

void PointRaster::erode(int width) {
    std::map<Eigen::Vector2i, std::vector<Eigen::Vector2d>, compareVector2i> newGrids;
    std::map<Eigen::Vector2i, double, compareVector2i> newGridsNum;

    for (const auto &[grid, count]: gridsNum_) {
        if (count > 0) {  // If the current grid is occupied
            bool allOccupied = true;
            for (int dx = -width; dx <= width; ++dx) {
                for (int dy = -width; dy <= width; ++dy) {
                    Eigen::Vector2i neighborGrid(grid[0] + dx, grid[1] + dy);
                    if (gridsNum_.find(neighborGrid) == gridsNum_.end() || gridsNum_[neighborGrid] == 0) {
                        allOccupied = false;
                        break;
                    }
                }
                if (!allOccupied) {
                    break;
                }
            }
            if (allOccupied) {
                newGrids[grid] = std::move(grids_[grid]);
                newGridsNum[grid] = count;
            }
        }
    }

    grids_ = std::move(newGrids);
    gridsNum_ = std::move(newGridsNum);
}

void PointRaster::dilate(int width) {
    std::map<Eigen::Vector2i, std::vector<Eigen::Vector2d>, compareVector2i> newGrids;
    std::map<Eigen::Vector2i, double, compareVector2i> newGridsNum;

    for (const auto &[grid, count]: gridsNum_) {
        if (count > 0) {
            for (int dx = -width; dx <= width; ++dx) {
                for (int dy = -width; dy <= width; ++dy) {
                    Eigen::Vector2i neighborGrid(grid[0] + dx, grid[1] + dy);
                    if (newGrids.find(neighborGrid) == newGrids.end()) {
                        newGrids[neighborGrid] = grids_.find(neighborGrid) != grids_.end() ? grids_[neighborGrid]
                                                                                           : std::vector<Eigen::Vector2d>();
                        newGridsNum[neighborGrid] = 0.8; //原始BIM点云投影得到的体素所代表的得分都要要相同
                    } else {
                        newGrids[neighborGrid] = grids_.find(neighborGrid) != grids_.end() ? grids_[neighborGrid]
                                                                                           : std::vector<Eigen::Vector2d>();
                        newGridsNum[neighborGrid] = std::max(newGridsNum[neighborGrid],
                                                             gridsNum_.find(neighborGrid) != gridsNum_.end()
                                                             ? gridsNum_[neighborGrid] : 0.0);
                    }
                }
            }
        }
    }

    grids_ = newGrids;
    gridsNum_ = newGridsNum;

    // Remove grids outside the bounds of the original raster
    for (auto it = grids_.begin(); it != grids_.end();) {
        const auto &grid = it->first;
        if (grid[0] < 0 || grid[1] < 0 || grid[0] >= size_[0] || grid[1] >= size_[1]) {
            it = grids_.erase(it);
            gridsNum_.erase(grid);
        } else {
            ++it;
        }
    }

}

void PointRaster::cv2DistanceTransform(double maxVal, double sigma) {
    cv::Mat img = toImage(true, 0);
    cv::Mat distTransform;
    cv::distanceTransform(img, distTransform, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    distTransform = cv::min(distTransform, maxVal);
//	for (int i = 0; i < distTransform.rows; ++i) {
//		for (int j = 0; j < distTransform.cols; ++j) {
//	std::cout << distTransform.at<float>(i, j) << " ";
//
//		}
//		std::cout << std::endl;
//	}
    auto applyGaussian = [](const cv::Mat &x, double miu, double sigma) {
        cv::Mat result = cv::Mat::zeros(x.size(), CV_64F);
        for (int i = 0; i < x.rows; ++i) {
            for (int j = 0; j < x.cols; ++j) {
                if (x.at<float>(i, j) > 0) {
                    result.at<double>(i, j) = std::exp(
                            -std::pow(x.at<float>(i, j) - miu, 2) / (2 * std::pow(sigma, 2)));
                }
            }
        }
        return result;
    };
	
	auto applyLinear = [](const cv::Mat &x, double miu, double bias) {
		cv::Mat result = cv::Mat::zeros(x.size(), CV_64F);
		for (int i = 0; i < x.rows; ++i) {
			for (int j = 0; j < x.cols; ++j) {
				if (x.at<float>(i, j) > 0) {
					result.at<double>(i, j) = x.at<float>(i, j) / miu + (1 - x.at<float>(i, j) / miu) * bias;
				}
			}
		}
		return result;
	};

//    distTransform = applyGaussian(distTransform, maxVal, sigma);
	distTransform = applyLinear(distTransform, maxVal, 0.4);

    for (auto &[grid, num]: gridsNum_) {
		if (num < 1) {
        num = distTransform.at<double>(grid[0], grid[1]);
		} //原始BIM点云投影得到的体素所代表的得分都要要相同
//		std::cout << "num: " << num << std::endl;
    }

}

cv::Mat PointRaster::toImage(bool uint8, int minNum) {
    cv::Mat gridImage = cv::Mat::zeros(size_[0], size_[1], CV_32F);
    for (const auto &[grid, num]: gridsNum_) {
        //		if (grid[0] < 0 || grid[1] < 0 || grid[0] >= size_[0] || grid[1] >= size_[1]) {
        //			continue;
        //		}
        if (num >= minNum) {
            gridImage.at<float>(grid[0], grid[1]) = num;
        } else if (num == -1) {
            //			std::cout   << grid[0] << "  " << grid[1] << "  " << "free" << std::endl;
            gridImage.at<float>(grid[0], grid[1]) = -1;
        }
    }
    if (uint8) {
        gridImage.convertTo(gridImage, CV_8U);
    }
    return gridImage;

}

//open3d::geometry::PointCloud PointRaster::toPointCloud(const std::string& type, const Eigen::Vector2d& color, int minNum) {
//	std::vector<Eigen::Vector2d> points;
//	for (const auto& [grid, num] : gridsNum) {
//		if ((type == "occupied" && num > minNum) || (type == "free" && num == -1)) {
//			points.push_back(gridToPoint(grid));
//		}
//	}
//	open3d::geometry::PointCloud pcd;
//	pcd.points_ = open3d::utility::Vector3dVector(points);
//	pcd.paint_uniform_color(color);
//	return pcd;
//}

void PointRaster::visualize(const std::vector<Eigen::Vector2d> *points) {
    cv::Mat gridImage = toImage();
    //scale 1.5 times
    cv::resize(gridImage, gridImage, cv::Size(), 6.0, 6.0, cv::INTER_NEAREST);
    // rotate -90 degrees
    cv::rotate(gridImage, gridImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imshow("Rasterized points", gridImage);
    cv::waitKey(0);
}


LineRaster::LineRaster(const std::vector<Eigen::Matrix2d> &lines, double gridSize) :
        lineset_(lines) {
    gridSize_ = gridSize;
    std::vector<Eigen::Vector2d> allPoints;
    for (const auto &line: lines) {
        for (int i = 0; i < line.rows(); ++i) {
            allPoints.emplace_back(line.row(i));
        }
    }
    this->initRaster(allPoints);
}

void LineRaster::LineRasterize(const std::vector<Eigen::Matrix2d> &lines) {
    const std::vector<Eigen::Matrix2d> &lines_used = lines.empty() ? lineset_ : lines;
    for (const auto &line: lines_used) {
        std::map<Eigen::Vector2i, std::pair<Eigen::Vector2d, double>, compareVector2i> grids;
        polylineGrids(line, grids);
        for (const auto &[key, value]: grids) {
            auto point = value.first;
            double score = value.second;
            if (score <= 0.0) continue;
            score = 1.0;
            if (gridsNum_.find(key) != gridsNum_.end()) {
                gridsNum_[key] += score;
                grids_[key].push_back({point});
            } else {
                gridsNum_[key] = score;
                grids_[key].push_back({point});
            }
        }
    }
    for (auto &[key, score]: gridsNum_) {
        if (score > 1) {
            score = 1;
        }
    }
}

Eigen::Vector2d pointToLineDistance(const Eigen::Vector2d &p, const std::array<Eigen::Vector2d, 2> &segment) {
    Eigen::Vector2d lineVec = segment[1] - segment[0];
    Eigen::Vector2d pntVec = p - segment[0];
    double u = pntVec.dot(lineVec) / lineVec.dot(lineVec);
    u = std::max(0.0, std::min(1.0, u));
    Eigen::Vector2d nearest = segment[0] + u * lineVec;
    return nearest;
}

void LineRaster::polylineGrids(const Eigen::Matrix2d &line,
                               std::map<Eigen::Vector2i, std::pair<Eigen::Vector2d, double>, compareVector2i> &grids) {
    Eigen::Vector2d p1 = line.row(0);
    Eigen::Vector2d p2 = line.row(1);
    //auto coords = plotLine(p1, p2);
    const auto &coords = drawLineOpenCV(p1, p2);
    for (const auto &coord: coords) {
        Eigen::Vector2i key = {std::get<0>(coord), std::get<1>(coord)};
        if (grids.find(key) != grids.end()) continue;
        Eigen::Vector2d gridCenter = gridToPoint(key);
        Eigen::Vector2d nearest = pointToLineDistance(gridCenter, {p1, p2});
        grids[key] = {nearest, std::get<2>(coord)};
    }
}

#include <opencv2/opencv.hpp>

std::vector<std::tuple<int, int, double>>
LineRaster::drawLineOpenCV(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, bool anti_aliasing) {
    /**
     * p1, p2: 2D points in the world frame
     * return: a list of tuples, each tuple contains (x, y, brightness)
     * */
    // convert the 2D point to the grid coordinate, note that the grid coordinate may be negative
    Eigen::Vector2i coord1 = pointToGrid({p1})[0];
    Eigen::Vector2i coord2 = pointToGrid({p2})[0];

    int margin = 10;
    int offset_x = std::min(coord1[0], coord2[0]) - margin;
    int offset_y = std::min(coord1[1], coord2[1]) - margin;

    coord1[0] -= offset_x;
    coord1[1] -= offset_y;
    coord2[0] -= offset_x;
    coord2[1] -= offset_y;

    int width = std::max(coord1[0], coord2[0]) + margin;
    int height = std::max(coord1[1], coord2[1]) + margin;
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC1);
    cv::line(img, cv::Point(coord1[0], coord1[1]), cv::Point(coord2[0], coord2[1]), cv::Scalar(255), 1,
             anti_aliasing ? cv::LINE_AA : cv::LINE_8);

    std::vector<std::tuple<int, int, double>> gridMap;
    // Extract the line pixels and their brightness
    for (int y = 0; y < img.rows; ++y) {
        for (int x = 0; x < img.cols; ++x) {
            if (img.at<uchar>(y, x) > 0) {
                double brightness = 1.0;
                if (anti_aliasing) {
                    brightness = img.at<uchar>(y, x) / 255.0;
                }
                gridMap.emplace_back(x + offset_x, y + offset_y, brightness);
            }
        }
    }
    return gridMap;
}

std::vector<std::tuple<int, int, double>>
LineRaster::plotLine(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
    double x1 = (p1.x() - origin_.x()) / gridSize_;
    double y1 = (p1.y() - origin_.y()) / gridSize_;
    double x2 = (p2.x() - origin_.x()) / gridSize_;
    double y2 = (p2.y() - origin_.y()) / gridSize_;

    auto ipart = [](double x) { return static_cast<int>(x); };
    auto round = [&](double x) { return ipart(x + 0.5); };
    auto fpart = [](double x) { return x - static_cast<int>(x); };
    auto rfpart = [&](double x) { return 1 - fpart(x); };

    std::vector<std::tuple<int, int, double>> gridMap;

    auto drawPixel = [&](int x, int y, double brightness) {
        gridMap.emplace_back(x, y, brightness);
    };

    bool steep = std::abs(y2 - y1) > std::abs(x2 - x1);
    if (steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }
    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    double dx = x2 - x1;
    double dy = y2 - y1;
    double gradient = dx == 0 ? 1 : dy / dx;

    int xend = round(x1);
    double yend = y1 + gradient * (xend - x1);
    double xgap = rfpart(x1 + 0.5);
    int xpxl1 = xend;
    int ypxl1 = ipart(yend);
    if (steep) {
        drawPixel(ypxl1, xpxl1, rfpart(yend) * xgap);
        drawPixel(ypxl1 + 1, xpxl1, fpart(yend) * xgap);
    } else {
        drawPixel(xpxl1, ypxl1, rfpart(yend) * xgap);
        drawPixel(xpxl1, ypxl1 + 1, fpart(yend) * xgap);
    }
    double intery = yend + gradient;

    xend = round(x2);
    yend = y2 + gradient * (xend - x2);
    xgap = fpart(x2 + 0.5);
    int xpxl2 = xend;
    int ypxl2 = ipart(yend);
    if (steep) {
        drawPixel(ypxl2, xpxl2, rfpart(yend) * xgap);
        drawPixel(ypxl2 + 1, xpxl2, fpart(yend) * xgap);
    } else {
        drawPixel(xpxl2, ypxl2, rfpart(yend) * xgap);
        drawPixel(xpxl2, ypxl2 + 1, fpart(yend) * xgap);
    }

    if (steep) {
        for (int x = xpxl1 + 1; x < xpxl2; ++x) {
            drawPixel(ipart(intery), x, rfpart(intery));
            drawPixel(ipart(intery) + 1, x, fpart(intery));
            intery += gradient;
        }
    } else {
        for (int x = xpxl1 + 1; x < xpxl2; ++x) {
            drawPixel(x, ipart(intery), rfpart(intery));
            drawPixel(x, ipart(intery) + 1, fpart(intery));
            intery += gradient;
        }
    }

    return gridMap;
}


void LineRaster::visualize(const std::vector<Eigen::Matrix2d> &lines) {
    cv::Mat gridImage = toImage(false, 0).t();
    cv::normalize(gridImage, gridImage, 0, 1, cv::NORM_MINMAX);
    cv::cvtColor(gridImage, gridImage, cv::COLOR_GRAY2BGR);

    std::vector<Eigen::Matrix2d> lines_vis = lines.empty() ? lineset_ : lines;

    for (const auto &line_vis: lines_vis) {
        Eigen::Matrix2d grids = (line_vis.rowwise() - origin_.transpose()) / gridSize_;

        for (int i = 0; i < grids.rows() - 1; ++i) {
            if (grids(i, 0) >= 0 && grids(i, 0) < gridImage.cols &&
                grids(i, 1) >= 0 && grids(i, 1) < gridImage.rows &&
                grids(i + 1, 0) >= 0 && grids(i + 1, 0) < gridImage.cols &&
                grids(i + 1, 1) >= 0 && grids(i + 1, 1) < gridImage.rows) {
                cv::line(gridImage,
                         cv::Point(grids(i, 0), grids(i, 1)),
                         cv::Point(grids(i + 1, 0), grids(i + 1, 1)),
                         cv::Scalar(255, 0, 0), 1);
            }
        }
    }
    cv::resize(gridImage, gridImage, cv::Size(size_[0] * 2.5, size_[1] * 2.5), 0, 0, cv::INTER_NEAREST);
    cv::flip(gridImage, gridImage, 0);
    cv::imshow("Rasterized lines", gridImage);
    cv::waitKey(0);
}