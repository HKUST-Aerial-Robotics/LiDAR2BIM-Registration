#include "utils/file_io.h"
#include <iostream>
#include <filesystem>
#include <system_error>
#include <boost/filesystem.hpp>

using namespace std;

namespace FileManager{
	bool CreateFile(std::ofstream &ofs, std::string file_path) {
		ofs.close();
		boost::filesystem::remove(file_path.c_str());
		
		ofs.open(file_path.c_str(), std::ios::out);
		if (!ofs) {
			std::cerr << "Cannot create file: " << file_path << std::endl;
			return false;
		}
		
		return true;
	}
	
	bool InitDirectory(std::string directory_path) {
		if (boost::filesystem::is_directory(directory_path)) {
			boost::filesystem::remove_all(directory_path);
		}
		
		return CreateDirectory(directory_path);
	}
	
	bool CreateDirectory(std::string directory_path) {
		if (!boost::filesystem::is_directory(directory_path)) {
			boost::filesystem::create_directory(directory_path);
		}
		
		if (!boost::filesystem::is_directory(directory_path)) {
			std::cerr << "Cannot create directory: " << directory_path << std::endl;
			return false;
		}
		
		return true;
	}
	void CreateRecursiveDirectory(const std::string &path) {
		try {
			if (path.empty()) {
				throw std::runtime_error("Path is empty.");
			}
			
			if (std::filesystem::exists(path)) {
				if (!std::filesystem::is_directory(path)) {
					throw std::runtime_error("Path exists but is not a directory: " + path);
				}
				return;
			}
			
			if (std::filesystem::create_directories(path)) {
				std::cout << "Directory created: " << path << std::endl;
			} else {
				std::cout << "Directory already exists: " << path << std::endl;
			}
		} catch (const std::exception &e) {
			std::cerr << "Error creating directory: " << e.what() << std::endl;
			std::abort();
		}
	
}
	int CountFiles(std::string directory_path, std::string suffix){
		int count = 0;
		boost::filesystem::path directory(directory_path);
		boost::filesystem::directory_iterator end_iter;
		for (boost::filesystem::directory_iterator iter(directory); iter != end_iter; iter++) {
			if (boost::filesystem::is_regular_file(iter->status())) {
				if (iter->path().extension() == suffix) {
					count++;
				}
			}
		}
		return count;
	}
	
	std::vector<std::string> GetFileNames(std::string directory_path, std::string suffix){
		std::vector<std::string> file_names;
		boost::filesystem::path directory(directory_path);
		boost::filesystem::directory_iterator end_iter;
		for (boost::filesystem::directory_iterator iter(directory); iter != end_iter; iter++) {
			if (boost::filesystem::is_regular_file(iter->status())) {
				if (iter->path().extension() == suffix) {
					file_names.push_back(iter->path().string());
				}
			}
		}
		return file_names;
	}
}
namespace fs = std::filesystem;

void mkdir(const std::string &path) {
	if (!fs::exists(path)) {
		std::cout << "Creating directory: " << path << std::endl;
		std::error_code ec;
		fs::create_directories(path, ec);
		if (ec) {
			std::cerr << "Error creating directory: " << ec.message() << std::endl;
		}
	}
}

void mkdirs(const std::vector<std::string> &paths) {
	for (const auto &path : paths) {
		mkdir(path);
	}
}

void clearDir(const std::string &path) {
	if (fs::exists(path) && fs::is_directory(path)) {
		std::cout << "Clearing directory: " << path << std::endl;
		for (const auto &entry : fs::directory_iterator(path)) {
			std::error_code ec;
			if (fs::is_regular_file(entry.status())) {
				fs::remove(entry.path(), ec);
			} else if (fs::is_directory(entry.status())) {
				fs::remove_all(entry.path(), ec);
			}
			if (ec) {
				std::cerr << "Error clearing directory: " << ec.message() << std::endl;
			}
		}
	} else {
		std::cerr << "Directory does not exist: " << path << std::endl;
	}
}

void clearDirs(const std::vector<std::string> &paths) {
	for (const auto &path : paths) {
		clearDir(path);
	}
}