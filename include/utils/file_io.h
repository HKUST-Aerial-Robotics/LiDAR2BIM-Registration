/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_FILE_IO_H
#define BIMREG_FILE_IO_H
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace FileManager {
	
	bool CreateFile(std::ofstream &ofs, std::string file_path);
	
	bool InitDirectory(std::string directory_path);
	
	bool CreateDirectory(std::string directory_path);
	
	void CreateRecursiveDirectory(const std::string &path);
	
	int CountFiles(std::string directory_path, std::string suffix);
	
	std::vector<std::string> GetFileNames(std::string directory_path, std::string suffix);
};

void mkdir(const std::string &path);

void mkdirs(const std::vector<std::string> &paths);

void clearDir(const std::string &path);

void clearDirs(const std::vector<std::string> &paths);

#endif //BIMREG_FILE_IO_H
