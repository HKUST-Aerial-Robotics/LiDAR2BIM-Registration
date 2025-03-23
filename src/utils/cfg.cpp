#include "utils/cfg.h"
#include <glog/logging.h>
#include "global_definition/global_definition.h"
#include "utils/file_io.h"
#include <filesystem>

namespace cfg {

    // benchmark config
    std::string project_path = bimreg::WORK_SPACE_PATH, config_file, seq, floor;
    double lengthRes = 0.5, angleRes = 3.0, xyRes = 0.5, yawRes = 1.0, nmsDist = 1.5, gridSize = 0.25, sdfDist = 0.5, sigma = 1.2, freeErodeWidth = 1.0, max_side_length = 40, lamda = 1.0, maxHeight = 1.9, minHeight =0.2, voxel_size = 0.01;
    int interval = 15, topL = 10000, topK = 5000, topJ = 150, NeighThreshold = 3;
	bool augmented = false;

    void readParameters(std::string config_file_) {
        std::cout << "-----------------config-----------------" << std::endl;
        config_file = config_file_;
        LOG(INFO) << "config_path: " << config_file;
        std::ifstream fin(config_file);
        if (!fin) {
            std::cout << "config_file: " << config_file << " not found." << std::endl;
            return;
        }

        YAML::Node config_node = YAML::LoadFile(config_file);
        max_side_length = get(config_node, "max_side_length", max_side_length);
        lengthRes = get(config_node, "length_res", lengthRes);
        angleRes = get(config_node, "angle_res", angleRes);
		augmented = get(config_node, "augmented", augmented);
        xyRes = get(config_node, "xy_res", xyRes);
        yawRes = get(config_node, "yaw_res", yawRes);
        nmsDist = get(config_node, "nms_dist", nmsDist);
        interval = get(config_node, "interval", interval);
        topL = get(config_node, "top_l", topL);
        topK = get(config_node, "top_k", topK);
        topJ = get(config_node, "top_j", topJ);
        gridSize = get(config_node, "grid_size", gridSize);
        sdfDist = get(config_node, "sdf_dist", sdfDist);
        sigma = get(config_node, "sigma", sigma);
        freeErodeWidth = get(config_node, "free_erode_width", freeErodeWidth);
		lamda = get(config_node, "lamda", lamda);
        seq = get(config_node, "seq", seq);
        floor = get(config_node, "floor", floor);
        NeighThreshold = static_cast<int>(std::round(nmsDist / xyRes));
		maxHeight = get(config_node, "pcd2d", "max_height", maxHeight);
		minHeight = get(config_node,"pcd2d", "min_height", minHeight);
		voxel_size = get(config_node, "voxel", voxel_size);
        std::cout << "----------------------------------------" << std::endl;

    }
}

void InitGLOG(std::string config_path) {
	std::string filename = std::filesystem::path(config_path).stem().string();
	std::string parent_path = std::filesystem::path(config_path).parent_path().string();
	const std::string base_path = "/configs/interval";
	
	std::string relative_path = parent_path.substr(bimreg::WORK_SPACE_PATH.size() + base_path.size());
	std::string log_dir = bimreg::WORK_SPACE_PATH + "/Log" + relative_path + "/" + filename + "/";
	FileManager::CreateRecursiveDirectory(log_dir);

    google::InitGoogleLogging("");


    std::replace(filename.begin(), filename.end(), '/', '_');
    std::string log_file = log_dir + "/" + filename + ".log.";
    google::SetLogDestination(google::INFO, log_file.c_str());

    FLAGS_log_dir = log_dir;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = false;
    FLAGS_logbufsecs = 0;
    //    FLAGS_timestamp_in_logfile_name = true;
}