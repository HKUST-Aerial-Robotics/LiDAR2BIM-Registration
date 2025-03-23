/*
 * @Description: file manager
 * @Author: Zhijian Qiao
 * @Date: 2020-02-24 19:22:53
 */
#ifndef TOOLS_FILE_MANAGER_HPP_
#define TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

namespace FileManager {

    bool CreateFile(std::ofstream &ofs, std::string file_path);

    bool InitDirectory(std::string directory_path);

    bool CreateDirectory(std::string directory_path);

    int CountFiles(std::string directory_path, std::string suffix);

    std::vector<std::string> GetFileNames(std::string directory_path, std::string suffix);
};

#endif
