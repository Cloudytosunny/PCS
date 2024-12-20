#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "PlaneClusteringSegmentation.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <experimental/filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <cstdlib>

namespace fs = std::experimental::filesystem;
namespace chrono = std::chrono;

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;
using KdTree = pcl::search::KdTree<pcl::PointXYZI>;
using KdTreePtr = KdTree::Ptr;

void create_directory_if_not_exists(const fs::path& dir_path) {
    if (!fs::exists(dir_path)) {
        if (fs::create_directory(dir_path)) {
            //std::cout << "Directory created: " << dir_path << std::endl;
        } else {
            //std::cerr << "Failed to create directory: " << dir_path << std::endl;
        }
    }
}

int
main(int argc, char** argv)
{
  try {
    std::vector<std::string> args(argv, argv + argc);
    if (args.size() != 2) {
      std::cerr << "usage: " << args.at(0)
                << " <pcd-file>" << std::endl;
      return 1;
    }

    fs::path pcd_path(args.at(1));

    PointCloudPtr cloud(new PointCloud);
    pcl::io::loadPCDFile(pcd_path.string(), *cloud);

    KdTreePtr tree(new KdTree);
    std::vector<pcl::PointIndices> clusters;

    PlaneClusteringSegmentation<pcl::PointXYZI> pcs;
    pcs.setInputCloud(cloud);
    pcs.setSearchMethod(tree);


    pcs.segment(clusters);

    std::cout << "#clusters: " << clusters.size() << std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pointcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr I_pointcloud(
      new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Test_pointcloud(
      new pcl::PointCloud<pcl::PointXYZI>);

    // 设置随机种子
    srand(time(NULL));

    // 生成随机数
    int random_r = rand() % 256;
    int random_g = rand() % 256;
    int random_b = rand() % 256;

    // double a = 0.0;
    // double b = 0.0;
    // double c = 0.0;
    // double d = 0.0;

    auto label = 0;
    for (const auto& cluster : clusters) {
      random_r = rand() % 256;
      random_g = rand() % 256;
      random_b = rand() % 256;

      Test_pointcloud->clear();
      for (auto index : cluster.indices) {
        auto& point = cloud->at(index);
        point.intensity = static_cast<float>(label);
        
        pcl::PointXYZRGB rgb_point;
        rgb_point.x = point.x;
        rgb_point.y = point.y;
        rgb_point.z = point.z;
        rgb_point.r = random_r;
        rgb_point.g = random_g;
        rgb_point.b = random_b;
        rgb_pointcloud->push_back(std::move(rgb_point));

        pcl::PointXYZI i_point;
        i_point.x = point.x;
        i_point.y = point.y;
        i_point.z = point.z;
        i_point.intensity = static_cast<float>(label);
        I_pointcloud->push_back(std::move(i_point));
        Test_pointcloud->push_back(std::move(i_point));
      }

      // std::cout << "label:" << label << " a:" << a << " b:" << b << " c:" << c << " d:" << d << std::endl;

      
      label++;
    }

    // std::cout << "pcd_path: " << pcd_path << std::endl;
    // std::cout << "pcd_path: " << pcd_path.string() << std::endl;
    // std::cout << "pcd_path: " << pcd_path.stem().string() << std::endl;
    // std::cout << "pcd_path: " << "pcd_results/" + pcd_path.stem().string() + "_clustered_rgb.pcd" << std::endl;

    create_directory_if_not_exists(pcd_path.parent_path() / "pcd_rgb");
    create_directory_if_not_exists(pcd_path.parent_path() / "pcd_i");

    pcl::io::savePCDFileASCII(
        fs::path(pcd_path)
            .replace_filename("pcd_rgb/" + pcd_path.stem().string() + "_clustered_rgb.pcd")
            .string(),
        *rgb_pointcloud);

    pcl::io::savePCDFileASCII(
        fs::path(pcd_path)
            .replace_filename("pcd_i/" + pcd_path.stem().string() + "_clustered_i.pcd")
            .string(),
        *I_pointcloud);

    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (...) {
    return 1;
  }
}
