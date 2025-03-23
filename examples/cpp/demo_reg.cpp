/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "utils/utils.h"
#include "utils/cfg.h"
#include "frontend/geometry/corner.h"
#include "frontend/geometry/lineseg.h"
#include "frontend/feature/descriptor.h"
#include "frontend/geometry/bim.h"
#include "frontend/geometry/submap.h"
#include "backend/reglib.h"
#include "global_definition/global_definition.h"


void vizResult(std::shared_ptr<SubmapManager> submap_ptr, std::shared_ptr<BIMManager> bim_ptr, FRGresult results) {
        Eigen::Matrix4d tf_est = results.tf * submap_ptr->GetPcd2d().GetT_rp();
        pcl::PointCloud<PointT>::Ptr pcd1 = submap_ptr->GetRawPcd();
        pcl::PointCloud<PointT>::Ptr pcd2 = bim_ptr->GetPcd3d();
        pcl::PointCloud<PointT>::Ptr pcd1_tf(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*pcd1, *pcd1_tf, tf_est.cast<float>());
        std::cout << "tf_est: " << tf_est << std::endl;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(255, 255, 255);
        viewer->addPointCloud(pcd1_tf, "pcd1_tf");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.929, 0.651,
                                                 "pcd1_tf");
        viewer->addPointCloud(pcd2, "pcd2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "pcd2");
        viewer->spin();
}

int main(int argc, char **argv) {
    double index;
    std::string config = "/configs/interval/15m/2F/building_day.yaml";
    if (argc == 2)
    {
        index = std::stod(argv[1]);
        if (index < 0 || index > 168)
        {
            std::cout << "Index out of range" << std::endl;
            return 0;
        }
    }
    else if (argc == 3)
    {
        index = std::stod(argv[2]);
        config = argv[1];
    }
    else
    {
        index =168;
    }
	std::string config_path = bimreg::WORK_SPACE_PATH + config;
	InitGLOG(config_path);
	cfg::readParameters(config_path);

	BIMManager bim;
	bim.LoadBim();
	SubmapManager submap(index);
	submap.LoadSubmap();
    submap.setT_rp();
    std::shared_ptr<BIMManager> bim_ptr = std::make_shared<BIMManager>(bim);
    std::shared_ptr<SubmapManager> submap_ptr = std::make_shared<SubmapManager>(submap);
    auto result = bimreg::GlobalRegistration(submap, bim);
    vizResult(submap_ptr, bim_ptr, result);
	if (bimreg::evaluate(result.tf, submap)) {
		std::cout << "Success" << std::endl;
	} else {
		std::cout << "Fail" << std::endl;
	}
	timer.printSummary();


	return 0;
}