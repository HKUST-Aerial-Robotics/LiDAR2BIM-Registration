/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_DESCRIPTOR_H
#define BIMREG_DESCRIPTOR_H

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include <stdexcept>
#include <typeinfo>
#include <cmath>
#include <tuple>
#include "frontend/geometry/corner.h"
#include "frontend/geometry/lineseg.h"

namespace geomset {


    struct Array6D {
        Array6D(const int &i, const int &i1, const int &i2, const int &i3, const int &i4, const int &i5) {
            data << i, i1, i2, i3, i4, i5;
        }

        Eigen::Array<int, 6, 1> data;

        bool operator==(const Array6D &other) const {
            return (data == other.data).all();
        }

        int operator[](int i) const {
            return data[i];
        }

        int &operator[](int i) {
            return data[i];
        }

        Array6D &operator=(const Array6D &other) {
            data = other.data;
            return *this;
        }

        Array6D() : data(Eigen::Array<int, 6, 1>::Zero()) {}

        bool operator==(const Array6D &other) {

            return (data == other.data).all();
        }

        bool operator!=(const Array6D &other) {
            return !(*this == other);
        }
		
		bool operator<(const Array6D &other) const {
			for (int i = 0; i < 6; ++i) {
				if (data[i] < other.data[i]) {
					return true;
				}
				else if (data[i] > other.data[i]) {
					return false;
				}
			}
			return false;
		}

		Array6D operator+(const Array6D &other) const {
			Array6D res;
			for (int i = 0; i < 6; ++i) {
				res[i] = data[i] + other[i];
			}
			return res;
		}

		Array6D operator-(const Array6D &other) const {
			Array6D res;
			for (int i = 0; i < 6; ++i) {
				res[i] = data[i] - other[i];
			}
			return res;
		}

		Array6D operator*(const Array6D &other) const {
			Array6D res;
			for (int i = 0; i < 6; ++i) {
				res[i] = data[i] * other[i];
			}
			return res;
		}

		Array6D operator/(const Array6D &other) const {
			Array6D res;
			for (int i = 0; i < 6; ++i) {
				res[i] = data[i] / other[i];
			}
			return res;
		}

        std::string toString() const {
            std::string str = "";
            for (int i = 0; i < 6; ++i) {
                str += std::to_string(data[i]) + " ";
            }
            return str;
        }

        auto begin() const -> decltype(data.data()) {
            return data.data();
        }

        auto end() const -> decltype(data.data() + data.size()) {
            return data.data() + data.size();
        }
    };

    struct Array6DHash {
        std::uint64_t operator()(const Array6D &a) const {
            std::uint64_t hash = 0;
            int s1 = 1;
            int s2 = s1 * 180;
            int s3 = s2 * 180;
            int s4 = s3 * 180;
            int s5 = s4 * 180;
            int s6 = s5 * 180;
            hash = a[0] * s1 + a[1] * s2 + a[2] * s3 + a[3] * s4 + a[4] * s5 + a[5] * s6;
            return hash;
        }
    };

    template<typename cornerType, typename HashStruc, typename HashMapping>
    class BaseDescHashTable {
    public:
        BaseDescHashTable(double lengthRes = 0.0, double angleRes = 0.0)
                : lengthRes_(lengthRes), angleRes_(angleRes) {}

        virtual ~BaseDescHashTable() = default;

        virtual std::vector<std::vector<cornerType>> buildGeomSet(const CornerVector &corners,
                                                                  const std::vector<std::shared_ptr<LineSegment>> &lineSegments) = 0;

        virtual void buildHashTable(const std::vector<std::vector<cornerType>> &geomSets, bool augmented = false) = 0;

        virtual HashStruc hash(const Eigen::Array<double, 6, 1> &feature) = 0;

        virtual const std::unordered_map<HashStruc, std::vector<Eigen::Matrix<double, 3, 2>>, HashMapping> &
        getHashTable() const {
            return hashTable_;
        }
	
		virtual const std::unordered_map<HashStruc, std::vector<int>, HashMapping> &
		getHashNeighTable() const {
			return hashNeighTable_;
		}

        virtual std::vector<std::tuple<HashStruc, HashStruc, int>>
        getCommonKeys(const BaseDescHashTable<cornerType, HashStruc, HashMapping> &tgtDescTable, int width = 1) = 0;

        void saveToFile(const std::string &filePath) const;
		void savehashNeighTable(const std::string &filePath) const;
		void savehashIntTable(const std::string &filePath) const;

        void loadFromFile(const std::string &filePath);
		void loadhashNeighTable(const std::string &filePath);
		void loadhashIntTable(const std::string &filePath);

        std::vector<Eigen::Matrix<double, 3, 2>> getGeomSets(const HashStruc &key) const;

        virtual void printInfo() const;

        using DescHash = std::unordered_map<HashStruc, std::vector<Eigen::Matrix<double, 3, 2>>, Array6DHash>;
		using DescNeighHash = std::unordered_map<HashStruc, std::vector<int>, Array6DHash>;
		using DescHashInt = std::unordered_map<int, std::vector<Eigen::Matrix<double, 3, 2>>>;
        DescHash hashTable_;
		DescNeighHash hashNeighTable_;
		DescHashInt hashIntTable_;
    protected:
        double lengthRes_;
        double angleRes_;

    };

    template<typename cornerType, typename HashStruc, typename HashMapping>
    void BaseDescHashTable<cornerType, HashStruc, HashMapping>::saveToFile(const std::string &filePath) const {
        std::ofstream outFile(filePath, std::ios::binary);
        if (!outFile) {
            throw std::runtime_error("Cannot open file for writing: " + filePath);
        }
        size_t size = hashTable_.size();
        outFile.write(reinterpret_cast<const char *>(&size), sizeof(size));
        for (const auto &pair: hashTable_) {
            const HashStruc &key = pair.first;
            size_t keySize = key.data.size();
            outFile.write(reinterpret_cast<const char *>(&keySize), sizeof(keySize));
            outFile.write(reinterpret_cast<const char *>(key.data.data()), keySize * sizeof(int));
            size_t vecSize = pair.second.size();
            outFile.write(reinterpret_cast<const char *>(&vecSize), sizeof(vecSize));
            for (const auto &mat: pair.second) {
                size_t rows = 3, cols = 2;
                outFile.write(reinterpret_cast<const char *>(mat.data()), sizeof(double) * rows * cols);
            }
        }
    }
	template<typename cornerType, typename HashStruc, typename HashMapping>
	void BaseDescHashTable<cornerType, HashStruc, HashMapping>::savehashNeighTable(const std::string &filePath) const {
		std::ofstream outFile(filePath, std::ios::binary);
		if (!outFile) {
			throw std::runtime_error("Cannot open file for writing: " + filePath);
		}
		size_t size = hashNeighTable_.size();
		outFile.write(reinterpret_cast<const char *>(&size), sizeof(size));
		for (const auto &pair : hashNeighTable_) {
			const HashStruc &key = pair.first;
			size_t keySize = key.data.size();
			outFile.write(reinterpret_cast<const char *>(&keySize), sizeof(keySize));
			outFile.write(reinterpret_cast<const char *>(key.data.data()), keySize * sizeof(int));
			size_t vecSize = pair.second.size();
			outFile.write(reinterpret_cast<const char *>(&vecSize), sizeof(vecSize));
			for (const auto &mat : pair.second) {
				outFile.write(reinterpret_cast<const char *>(&mat), sizeof(int));
			}
		}
		
	}
	template<typename cornerType, typename HashStruc, typename HashMapping>
	void BaseDescHashTable<cornerType, HashStruc, HashMapping>::savehashIntTable(const std::string &filePath) const {
		std::ofstream outFile(filePath, std::ios::binary);
		if (!outFile) {
			throw std::runtime_error("Cannot open file for writing: " + filePath);
		}
		size_t size = hashIntTable_.size();
		outFile.write(reinterpret_cast<const char *>(&size), sizeof(size));
		for (const auto &pair : hashIntTable_) {
			const int &key = pair.first;
			outFile.write(reinterpret_cast<const char *>(&key), sizeof(key));
			size_t vecSize = pair.second.size();
			outFile.write(reinterpret_cast<const char *>(&vecSize), sizeof(vecSize));
			for (const auto &mat : pair.second) {
				size_t rows = 3, cols = 2;
				outFile.write(reinterpret_cast<const char *>(mat.data()), sizeof(double) * rows * cols);
			}
		}
	}
	
    template<typename cornerType, typename HashStruc, typename HashMapping>
    void BaseDescHashTable<cornerType, HashStruc, HashMapping>::loadFromFile(const std::string &filePath) {
        std::ifstream inFile(filePath, std::ios::binary);
        if (!inFile) {
            throw std::runtime_error("Cannot open file for reading: " + filePath);
        }
        size_t size;
        inFile.read(reinterpret_cast<char *>(&size), sizeof(size));
        for (size_t i = 0; i < size; ++i) {
            size_t keySize;
            inFile.read(reinterpret_cast<char *>(&keySize), sizeof(keySize));
            HashStruc key;
            if (keySize != key.data.size()) {
                throw std::runtime_error("Key size mismatch");
            }
            inFile.read(reinterpret_cast<char *>(key.data.data()), keySize * sizeof(int));

            size_t vecSize;
            inFile.read(reinterpret_cast<char *>(&vecSize), sizeof(vecSize));
            std::vector<Eigen::Matrix<double, 3, 2>> vec;
            for (size_t j = 0; j < vecSize; ++j) {
                size_t rows = 3, cols = 2;
                Eigen::Matrix<double, 3, 2> mat;
                inFile.read(reinterpret_cast<char *>(mat.data()), sizeof(double) * rows * cols);
                vec.push_back(mat);
            }
            hashTable_[key] = vec;
        }
    }
	
	template<typename cornerType, typename HashStruc, typename HashMapping>
	void BaseDescHashTable<cornerType, HashStruc, HashMapping>::loadhashNeighTable(const std::string &filePath) {
		std::ifstream inFile(filePath, std::ios::binary);
		if (!inFile) {
			throw std::runtime_error("Cannot open file for reading: " + filePath);
		}
		size_t size;
		inFile.read(reinterpret_cast<char *>(&size), sizeof(size));
		for (size_t i = 0; i < size; ++i) {
			size_t keySize;
			inFile.read(reinterpret_cast<char *>(&keySize), sizeof(keySize));
			HashStruc key;
			if (keySize != key.data.size()) {
				throw std::runtime_error("Key size mismatch");
			}
			inFile.read(reinterpret_cast<char *>(key.data.data()), keySize * sizeof(int));

			size_t vecSize;
			inFile.read(reinterpret_cast<char *>(&vecSize), sizeof(vecSize));
			std::vector<int> vec;
			for (size_t j = 0; j < vecSize; ++j) {
				int mat;
				inFile.read(reinterpret_cast<char *>(&mat), sizeof(int));
				vec.push_back(mat);
			}
			hashNeighTable_[key] = vec;
		}
	}
	template<typename cornerType, typename HashStruc, typename HashMapping>
	void BaseDescHashTable<cornerType, HashStruc, HashMapping>::loadhashIntTable(const std::string &filePath) {
		std::ifstream inFile(filePath, std::ios::binary);
		if (!inFile) {
			throw std::runtime_error("Cannot open file for reading: " + filePath);
		}
		size_t size;
		inFile.read(reinterpret_cast<char *>(&size), sizeof(size));
		for (size_t i = 0; i < size; ++i) {
			int key;
			inFile.read(reinterpret_cast<char *>(&key), sizeof(key));
			size_t vecSize;
			inFile.read(reinterpret_cast<char *>(&vecSize), sizeof(vecSize));
			std::vector<Eigen::Matrix<double, 3, 2>> vec;
			for (size_t j = 0; j < vecSize; ++j) {
				size_t rows = 3, cols = 2;
				Eigen::Matrix<double, 3, 2> mat;
				inFile.read(reinterpret_cast<char *>(mat.data()), sizeof(double) * rows * cols);
				vec.push_back(mat);
			}
			hashIntTable_[key] = vec;
		}
	}
	

    template<typename cornerType, typename HashStruc, typename HashMapping>
    std::vector<Eigen::Matrix<double, 3, 2>>
    BaseDescHashTable<cornerType, HashStruc, HashMapping>::getGeomSets(const HashStruc &key) const {
        auto it = hashTable_.find(key);
        if (it != hashTable_.end()) {
            return it->second;
        }
        return std::vector<Eigen::Matrix<double, 3, 2>>();
    }

    template<typename cornerType, typename HashStruc, typename HashMapping>
    void BaseDescHashTable<cornerType, HashStruc, HashMapping>::printInfo() const {
        std::cout << "--------------------------------" << std::endl;
        std::cout << "HashTable Info:" << std::endl;
        std::cout << "Length Resolution: " << lengthRes_ << std::endl;
        std::cout << "Angle Resolution: " << angleRes_ << std::endl;
        std::cout << "Number of Keys: " << hashTable_.size() << std::endl;

        size_t totalNum = 0;
        for (const auto &pair: hashTable_) {
            totalNum += pair.second.size();
        }
        std::cout << "Total Number of GeomSets: " << totalNum << std::endl;
        std::cout << "--------------------------------" << std::endl;
    }


    class TriDescHashTable : public BaseDescHashTable<CornerPtr, Array6D, Array6DHash> {
    public:
        TriDescHashTable(double lengthRes = 0.0, double angleRes = 0.0, double radius = 40.0)
                : BaseDescHashTable(lengthRes, angleRes), radius_(radius) {};

        CornerTriplets 	buildTriplets(const CornerVector &corners);

        CornerTriplets buildGeomSet(const CornerVector &corners,
                                    const std::vector<std::shared_ptr<LineSegment>> &lineSegments) override;

        bool isValidTriangle(const std::vector<double> &sides) const;

        Eigen::Array<double, 6, 1> descriptorExtraction(const std::vector<double> &sides,
                                                        const CornerPtr &A,
                                                        const CornerPtr &B,
                                                        const CornerPtr &C);

        void buildHashTable(const CornerTriplets &geomSets, bool augmented = false) override;

        std::vector<Array6D> getKeys() const;

        const std::unordered_map<Array6D, std::vector<Eigen::Matrix<double, 3, 2>>, Array6DHash> &
        getHashTable() const override {
            return hashTable_;
        }

        Array6D hash(const Eigen::Array<double, 6, 1> &feature) override;

        std::vector<std::tuple<Array6D, Array6D, int>>
        getCommonKeys(const BaseDescHashTable<CornerPtr, Array6D, Array6DHash> &tgtDescTable, int width = 1) override;
		std::vector<std::tuple<Array6D, int, int>>
		getAugCommonKeys(const BaseDescHashTable<CornerPtr, Array6D, Array6DHash> &tgtDescTable);
    protected:
        double radius_;
        double cosThreshold = std::cos(M_PI / 90.0);
    };

    class TriDescHashTable2 : public BaseDescHashTable<CornerPtr, std::string, Array6DHash> {
    public:
        TriDescHashTable2(double lengthRes = 0.0, double angleRes = 0.0, double radius = 40.0)
                : BaseDescHashTable(lengthRes, angleRes), radius_(radius) {
            triDescHashTable = TriDescHashTable(lengthRes, angleRes, radius);
        };

        CornerTriplets buildGeomSet(const CornerVector &corners,
                                    const std::vector<std::shared_ptr<LineSegment>> &lineSegments) override;


        void buildHashTable(const CornerTriplets &geomSets, bool augmented = false) override;


        const std::unordered_map<std::string, std::vector<Eigen::Matrix<double, 3, 2>>> &
        getStringHashTable() const {
            return stringHashTable_;
        }


        std::string hash(const Eigen::Array<double, 6, 1> &feature) override;

        std::vector<std::tuple<std::string, std::string, int>>
        getCommonKeys(const BaseDescHashTable<CornerPtr, std::string, Array6DHash> &tgtDescTable,
                      int width = 1) override;

        std::vector<std::pair<std::string, std::string>>
        getStringCommonKeys(const TriDescHashTable2 &tgtDescTable, int width = 1) const;

        void printInfo() const override;

    protected:
        double radius_;
        double cosThreshold = std::cos(M_PI / 90.0);
        TriDescHashTable triDescHashTable;
        std::unordered_map<std::string, std::vector<Eigen::Matrix<double, 3, 2>>> stringHashTable_;
    };
}
#endif //BIMREG_DESCRIPTOR_H
