/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_RASTER_UTILS_H
#define BIMREG_RASTER_UTILS_H

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
//#include <open3d/Open3D.h>
#include "utils/cfg.h"

class PointRaster {
public:
    PointRaster(double gridSize = 0.5);

    PointRaster(const std::vector<Eigen::Vector2d> &points, double gridSize = 0.5);

    void rasterize(const std::vector<Eigen::Vector2d> &points = std::vector<Eigen::Vector2d>(), double invert = 1,
                   bool override = false, int width = 0);
	void rasterize_free(const std::vector<Eigen::Vector2d> &points = std::vector<Eigen::Vector2d>(), double invert = 1,
				   bool override = false, int width = 0);
    std::vector<Eigen::Vector2i> pointToGrid(const std::vector<Eigen::Vector2d> &points, int width = 0);

    Eigen::Vector2d gridToPoint(const Eigen::Vector2i &grid);

    double getObsNum(const Eigen::Vector2i &key, int width = 0);

    std::vector<Eigen::Vector2d> getCenters(const std::vector<Eigen::Vector2i> &grids = {});

    void erode(int width = 1);

    void dilate(int width = 1);

    void cv2DistanceTransform(double maxVal, double sigma);
	
    cv::Mat toImage(bool uint8 = false, int minNum = 0);

    //	open3d::geometry::PointCloud toPointCloud(const std::string& type = "occupied", const Eigen::Vector2d& color = {1, 0, 0}, int minNum = 0);
    void visualize(const std::vector<Eigen::Vector2d> *points = nullptr);

    struct compareVector2i {
        bool operator()(const Eigen::Vector2i &a, const Eigen::Vector2i &b) const {
            return std::tie(a[0], a[1]) < std::tie(b[0], b[1]);
        }
    };

    std::map<Eigen::Vector2i, std::vector<Eigen::Vector2d>, compareVector2i> grids_;
    std::map<Eigen::Vector2i, double, compareVector2i> gridsNum_;
protected:
    void initRaster(const std::vector<Eigen::Vector2d> &points);

    std::vector<Eigen::Vector2d> points_;
    double gridSize_;
    Eigen::Vector2d origin_;
    Eigen::Vector2i size_;

};


class LineRaster : public PointRaster {
public:
    LineRaster(const std::vector<Eigen::Matrix2d> &lines, double gridSize = 0.5);

    void LineRasterize(const std::vector<Eigen::Matrix2d> &lines = std::vector<Eigen::Matrix2d>());

    void polylineGrids(const Eigen::Matrix2d &line,
                       std::map<Eigen::Vector2i, std::pair<Eigen::Vector2d, double>, compareVector2i> &grids);

    std::vector<std::tuple<int, int, double>> plotLine(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

    std::vector<std::tuple<int, int, double>>
    drawLineOpenCV(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, bool anti_aliasing = true);

    void visualize(const std::vector<Eigen::Matrix2d> &lines = std::vector<Eigen::Matrix2d>());

private:
    std::vector<Eigen::Matrix2d> lineset_;
};

#endif //BIMREG_RASTER_UTILS_H
