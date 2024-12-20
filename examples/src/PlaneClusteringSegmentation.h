/*
 * Copyright (c) 2024 Wang Shaohu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <queue>

#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

template <typename PointT>
class PlaneClusteringSegmentation : public pcl::PCLBase<PointT> {
  using Base = pcl::PCLBase<PointT>;
  using Base::deinitCompute;
  using Base::indices_;
  using Base::initCompute;
  using Base::input_;

public:

  int rows = 1800;
  int cols = 16;

  float T = 160;  
  float Y = 170;  

  float T_h_max = 170;  
  float T_h_min = 165;  
  float D_h = 50;  
  float w1_h = 0.20, w2_h = 0.20, w3_h = 0.20, w4_h = 0.20, w5_h = 0.20;
  //float w1_h = 0.3, w2_h = 0.34, w3_h = 0.33, w4_h = 0.00, w5_h = 0.00;
  // float w1_h = 1.00, w2_h = 0.00, w3_h = 0.00, w4_h = 0.00, w5_h = 0.00;

  float T_v_max = 170;  
  float T_v_min = 160;  
  float D_v = 50;  
  float w1_v = 0.33, w2_v = 0.34, w3_v = 0.33, w4_v = 0.00, w5_v = 0.00; 
  // float w1_v = 0.26, w2_v = 0.23, w3_v = 0.20, w4_v = 0.17, w5_v = 0.14; 

  float fusion_angle = 25;
  float fusion_dis = 0.05;


  int fusion[100]; 

  int lineIDX[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
  double Aplane[8000], Bplane[8000], Cplane[8000],Dplane[8000],Eplane[8000],Fplane[8000], Gplane[8000], label_1[8000], label_2[8000];

  using KdTree = pcl::search::Search<PointT>;
  using KdTreePtr = typename KdTree::Ptr;
  using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;

  PlaneClusteringSegmentation()
  : cluster_tolerance_(0.0)
  , max_cluster_size_(std::numeric_limits<size_t>::max())
  , min_cluster_size_(1)
  , quality_(0.0)
  , tree_()
  {}


  typename pcl::PointCloud<PointT>::Ptr fullCloud;
  typename pcl::PointCloud<PointT>::Ptr fullInfoCloud; 
  PointT nanPoint;

  double
  getClusterTolerance() const
  {
    return cluster_tolerance_;
  }

  void
  setClusterTolerance(double tolerance)
  {
    cluster_tolerance_ = tolerance;
  }

  size_t
  getMaxClusterSize() const
  {
    return max_cluster_size_;
  }

  void
  setMaxClusterSize(size_t max_cluster_size)
  {
    max_cluster_size_ = max_cluster_size;
  }

  size_t
  getMinClusterSize() const
  {
    return min_cluster_size_;
  }

  void
  setMinClusterSize(size_t min_cluster_size)
  {
    min_cluster_size_ = min_cluster_size;
  }

  double
  getQuality() const
  {
    return quality_;
  }

  void
  setQuality(double quality)
  {
    quality_ = quality;
  }

  KdTreePtr
  getSearchMethod() const
  {
    return (tree_);
  }

  void
  setSearchMethod(const KdTreePtr& tree)
  {
    tree_ = tree;
  }

  void projectPointCloud(){

      nanPoint.x = std::numeric_limits<float>::quiet_NaN();
      nanPoint.y = std::numeric_limits<float>::quiet_NaN();
      nanPoint.z = std::numeric_limits<float>::quiet_NaN();
      nanPoint.intensity = -1;

      // range image projection
      float verticalAngle, horizonAngle, range;
      int rowIdn, columnIdn, index, cloudSize; 
      PointT thisPoint;

      float ang_res_x = 0.2;
      float ang_res_y = 2.0;
      float ang_bottom = 15.0+0.1;

      fullCloud.reset(new pcl::PointCloud<PointT>());
      fullInfoCloud.reset(new pcl::PointCloud<PointT>());

      fullCloud->points.resize(cols*rows);
      fullInfoCloud->points.resize(cols*rows);

      std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
      std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);

      cloudSize = input_->size();

      for (int i = 0; i < cloudSize; ++i){

          thisPoint.x = input_->at(i).x;
          thisPoint.y = input_->at(i).y;
          thisPoint.z = input_->at(i).z;
          // find the row and column index in the iamge for this point

          verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
          rowIdn = (verticalAngle + ang_bottom) / ang_res_y;

          if (rowIdn < 0 || rowIdn >= cols)
              continue;

          horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

          columnIdn = -round((horizonAngle-90.0)/ang_res_x) + rows/2;
          if (columnIdn >= rows)
              columnIdn -= rows;

          if (columnIdn < 0 || columnIdn >= rows)
              continue;

          range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
          // if (range < sensorMinimumRange)
          //     continue;
          
          // rangeMat.at<float>(rowIdn, columnIdn) = range;

          thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

          index = columnIdn  + rowIdn * rows;
          fullCloud->points[index] = thisPoint;
          fullInfoCloud->points[index] = thisPoint;
          fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
          
      }
      // input_ = fullCloud;
  }

  
double distance(const PointT& p1, const PointT& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}


double average_spacing(const PointT& point0, const PointT& point1,
                       const PointT& point2, const PointT& point3,
                       const PointT& point4, const PointT& point5) {
    double total_distance = 0.0;
    total_distance += distance(point0, point1);
    total_distance += distance(point1, point2);
    total_distance += distance(point2, point3);
    total_distance += distance(point3, point4);
    total_distance += distance(point4, point5);
    return total_distance / 5.0;  
}

  double cal_C(PointT p1,PointT p2,PointT p3){
            double  diffX1 = p1.x - p2.x;
            double  diffY1 = p1.y - p2.y;
            double  diffZ1 = p1.z - p2.z;

            double  diffX2 = p3.x - p2.x;
            double  diffY2 = p3.y - p2.y;
            double  diffZ2 = p3.z - p2.z;

            if((abs(diffX1)<0.001&&abs(diffY1)<0.001&&abs(diffZ1)<0.001) || (abs(diffX2)<0.001&&abs(diffY2)<0.001&&abs(diffZ2)<0.001)){
                return 0;
            }

            double C1 = (diffX1 * diffX2 + diffY1 * diffY2 + diffZ1 * diffZ2) / (sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1) * (sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2)));

            return acos(C1)* 180 / M_PI;
  }


  float calRotation(Eigen::Vector3f u, Eigen::Vector3f v)
  {
    float angle = acos(u.dot(v) / (u.norm()*v.norm()));
    if (angle > M_PI / 2)
    {
      u = -u;
      angle = M_PI - angle;
    }
    return angle * 180/M_PI;
  }

  void get_plane_ransac(const pcl::PointCloud<PointT> & src_cloud,double &a,double &b,double &c,double &d)
  { 
      typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
      Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();

      pcl::transformPointCloud(src_cloud, *filtered, tilt_matrix);


      if(src_cloud.width > 4)
      {

          // RANSAC
          typename pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
          pcl::RandomSampleConsensus<PointT> ransac(model_p);
          ransac.setDistanceThreshold(0.1);  
          ransac.computeModel();

          pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
          ransac.getInliers(inliers->indices);

          Eigen::VectorXf coeffs;
          ransac.getModelCoefficients(coeffs);
          a = coeffs[0];
          b = coeffs[1];
          c = coeffs[2];
          d = coeffs[3];
      }
      else
      {
          a = 0;
          b = 0;
          c = 1;
          d = 0;
      }
  }

  float calculateHorizontalCurvature(int x, int y) {
    if (x < 0 || x > cols - 1) {
        return -1; 
    }


    if (y < 0) {
        y = rows + y;
    }
    if (y >= rows) {
        y = y - rows;
    }

    if(y*cols+x > rows*cols){
      std::cout << "1???" << y*cols+x << std::endl;
    }

    PointT current_point = input_->at(y*cols+lineIDX[x]);
    if (y - 1 < 0) {y = rows + y;}
    PointT left_point_1 = input_->at((y - 1)*cols+lineIDX[x]);
    if (y + 1 >= rows) { y = y - rows;}
    PointT right_point_1 = input_->at((y + 1)*cols+lineIDX[x]);

    if (y - 2 < 0) {y = rows + y;}
    PointT left_point_2 = input_->at((y - 2)*cols+lineIDX[x]);
    if (y + 2 >= rows) { y = y - rows;}
    PointT right_point_2 = input_->at((y + 2)*cols+lineIDX[x]);

    if (y - 3 < 0) {y = rows + y;}
    PointT left_point_3 = input_->at((y - 3)*cols+lineIDX[x]);
    if (y + 3 >= rows) { y = y - rows;}
    PointT right_point_3 = input_->at((y + 3)*cols+lineIDX[x]);

    if (y - 4 < 0) {y = rows + y;}
    PointT left_point_4 = input_->at((y - 4)*cols+lineIDX[x]);
    if (y + 4 >= rows) { y = y - rows;}
    PointT right_point_4 = input_->at((y + 4)*cols+lineIDX[x]);

    if (y - 5 < 0) {y = rows + y;}
    PointT left_point_5 = input_->at((y - 5)*cols+lineIDX[x]);
    if (y + 5 >= rows) { y = y - rows;}
    PointT right_point_5 = input_->at((y + 5)*cols+lineIDX[x]);

    // float diffX = w1 * (right_point_1.x + left_point_1.x - 2* current_point.x) + w2 * (right_point_2.x + left_point_2.x - 2* current_point.x) + w3 * (right_point_3.x + left_point_3.x - 2* current_point.x) + w4 * (right_point_4.x + left_point_4.x - 2* current_point.x) + w5 * (right_point_5.x + left_point_5.x - 2* current_point.x);
    // float diffY = w1 * (right_point_1.y + left_point_1.y - 2* current_point.y) + w2 * (right_point_2.y + left_point_2.y - 2* current_point.y) + w3 * (right_point_3.y + left_point_3.y - 2* current_point.y) + w4 * (right_point_4.y + left_point_4.y - 2* current_point.y) + w5 * (right_point_5.y + left_point_5.y - 2* current_point.y);
    // float diffZ = w1 * (right_point_1.z + left_point_1.z - 2* current_point.z) + w2 * (right_point_2.z + left_point_2.z - 2* current_point.z) + w3 * (right_point_3.z + left_point_3.z - 2* current_point.z) + w4 * (right_point_4.z + left_point_4.z - 2* current_point.z) + w5 * (right_point_5.z + left_point_5.z - 2* current_point.z);

    if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 1 ){
        w1_h = 0.0, w2_h = 0.0, w3_h = 0.0, w4_h = 0.0, w5_h = 1;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 1 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 2){
        w1_h = 0.0, w2_h = 0.0, w3_h = 0.0, w4_h = 0.5, w5_h = 0.5;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 2 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 3){
        w1_h = 0.0, w2_h = 0.0, w3_h = 0.33, w4_h = 0.34, w5_h = 0.33;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 3 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 10){
        w1_h = 0.0, w2_h = 0.0, w3_h = 0.33, w4_h = 0.34, w5_h = 0.33;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 10 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 50){
        w1_h = 0.0, w2_h = 0.5, w3_h = 0.5, w4_h = 0.0, w5_h = 0.0;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 50){
        w1_h = 0.5, w2_h = 0.5, w3_h = 0.0, w4_h = 0.0, w5_h = 0.0;
    }

    float horizontal_curvature = w1_h * cal_C(right_point_1, current_point, left_point_1) + w2_h * cal_C(right_point_2, current_point, left_point_2) + w3_h * cal_C(right_point_3, current_point, left_point_3) + w4_h * cal_C(right_point_4, current_point, left_point_4) + w5_h * cal_C(right_point_5, current_point, left_point_5);


    // double r_avg_spacing = average_spacing(current_point, right_point_1, right_point_2, right_point_3, right_point_4, right_point_5);
    // double l_avg_spacing = average_spacing(current_point, left_point_1, left_point_2, left_point_3, left_point_4, left_point_5);
    // if(l_avg_spacing/r_avg_spacing > 3 || l_avg_spacing/r_avg_spacing < 0.3){
    //     horizontal_curvature = 60;
    // }

    // float diffX = left_point_1.x + right_point_1.x - 2* current_point.x;
    // float diffY = left_point_1.y + right_point_1.y - 2* current_point.y;
    // float diffZ = left_point_1.z + right_point_1.z - 2* current_point.z;

    
    // float horizontal_curvature = diffX * diffX + diffY * diffY + diffZ * diffZ;

    T = (M_PI-atan((0.02*cos((2*M_PI)/(double)cols)+0.02)/((0.02+std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z))*sin((2*M_PI)/(double)cols))))/M_PI*180-(T_h_max-160);

    return horizontal_curvature;
  }

  float calculateVerticalCurvature(int x, int y) {
    
    // float w1 = 0.26, w2 = 0.23, w3 = 0.20, w4 = 0.17, w5 = 0.14;

    // float w1 = 0.17, w2 = 0.17, w3 = 0.20, w4 = 0.23, w5 = 0.23;

    // float w1 = 0.43, w2 = 0.23, w3 = 0.34, w4 = 0.00, w5 = 0.00;

    if (x <= 0 || x >= cols - 1) {
        return 180;  
    }

    if (y < 0) {
        y = rows + y;
    }
    if (y >= rows) {
        y = y - rows;
    }
    if(y*cols+x > rows*cols){
      // std::cout << "2???" << y*cols+x << std::endl;
    }

    
    PointT current_point = input_->at(y*cols+lineIDX[x]);

    if (x+1 >= cols) {return -1;} // x=15
    PointT up_point_1 = input_->at(y*cols+lineIDX[x+1]);
    if (x-1 < 0) {return -1;} // x = 0
    PointT down_point_1 = input_->at(y*cols+lineIDX[x-1]);

    if (x+2 >= cols) {x=13;}  //x = 14
    PointT up_point_2 = input_->at(y*cols+lineIDX[x+2]);
    if (x-2 < 0) {x=2;}  // x = 1
    PointT down_point_2 = input_->at(y*cols+lineIDX[x-2]);

    if (x+3 >= cols) {x=12;}  // x = 13
    PointT up_point_3 = input_->at(y*cols+lineIDX[x+3]);
    if (x-3 < 0) {x=3;}  // x = 2
    PointT down_point_3 = input_->at(y*cols+lineIDX[x-3]);

    if (x+4 >= cols) {x=11;}  // x = 12
    PointT up_point_4 = input_->at(y*cols+lineIDX[x+4]);
    if (x-4 < 0) {x=4;}  // x = 3
    PointT down_point_4 = input_->at(y*cols+lineIDX[x-4]);

    if (x+5 >= cols) {x=10;}  //x = 11
    PointT up_point_5 = input_->at(y*cols+lineIDX[x+5]);
    if (x-5 < 0) {x=5;}  //x = 4
    PointT down_point_5 = input_->at(y*cols+lineIDX[x-5]);

    // float diffX = up_point.x + down_point.x - 2* current_point.x;
    // float diffY = up_point.y + down_point.y - 2* current_point.y;
    // float diffZ = up_point.z + down_point.z - 2* current_point.z;

    // // 
    // float vertical_curvature = diffX * diffX + diffY * diffY + diffZ * diffZ;

      if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 1 ){
        w1_v = 0.0, w2_v = 0.0, w3_v = 0.0, w4_v = 0.0, w5_v = 1;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 1 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 2){
        w1_v = 0.0, w2_v = 0.0, w3_v = 0.0, w4_v = 0.5, w5_v = 0.5;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 2 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 3){
        w1_v = 0.0, w2_v = 0.5, w3_v = 0.5, w4_v = 0.0, w5_v = 0.0;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 3 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 10){
        w1_v = 0.5, w2_v = 0.5, w3_v = 0.0, w4_v = 0.0, w5_v = 0.0;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 10 && std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) < 50){
        w1_v = 0.5, w2_v = 0.5, w3_v = 0.0, w4_v = 0.0, w5_v = 0.0;
    }else if(std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z) >= 50){
        w1_v = 1, w2_v = 0.0, w3_v = 0.0, w4_v = 0.0, w5_v = 0.0;
    }

    float vertical_curvature = w1_v * cal_C(up_point_1, current_point, down_point_1) + w2_v * cal_C(up_point_2, current_point, down_point_2) + w3_v * cal_C(up_point_3, current_point, down_point_3) + w4_v * cal_C(up_point_4, current_point, down_point_4) + w5_v * cal_C(up_point_5, current_point, down_point_5);

    // double u_avg_spacing = average_spacing(current_point, up_point_1, up_point_2, up_point_3, up_point_4, up_point_5);
    // double d_avg_spacing = average_spacing(current_point, down_point_1, down_point_2, down_point_3, down_point_4, down_point_5);
    // if(u_avg_spacing/d_avg_spacing > 3 || u_avg_spacing/d_avg_spacing < 0.3){
    //     vertical_curvature = 60;
    // }

    Y = (M_PI-atan((0.02*cos(2.0/180*M_PI)+0.02)/((0.02+std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z))*sin(2.0/180*M_PI))))/M_PI*180-(T_v_max-165);


    return vertical_curvature;
  }

  void addNeighborsToQueue(int x, int y, std::queue<std::pair<int, int>>& seed_queue) {


      if(y*cols+x > rows*cols){
        // std::cout << "3???" << y*cols+x << std::endl;
        return;
      }

      if (x < 0 || x > 15) {
        return;  // 
      }

      if (y < 0){
          y = rows + y;
      }
      if (y >= rows && y < 3599) {
          y = y - rows;
      }
      // if(x < (-1*1799)){
      //   return;
      // }
      if(y > 3599){
        std::cout << "88???" << y << std::endl;
        return;
      }

      seed_queue.push({x, y});
  }

  void
  segment(std::vector<pcl::PointIndices>& clusters)
  {
    for (int i = 0; i < 8000; i++)
    {
        Aplane[i] = 0;
        Bplane[i] = 0;
        Cplane[i] = 0;
        Dplane[i] = 0;
        Eplane[i] = 0;
        Fplane[i] = 0;
        Gplane[i] = 0;
        label_1[i] = 0;
        label_2[i] = 0;
    }

    // projectPointCloud();

    clusters.clear();

    if (!initCompute() || input_->empty() || indices_->empty())
      return;


    std::vector<std::vector<int>> labels(rows, std::vector<int>(cols, -1));
    std::queue<std::pair<int, int>> seed_queue;

    std::vector<bool> removed(input_->size(), false);

    int label = 0;
    // int label1 = 1;
    // int label2 = 1;
    Graph g;

    pcl::PCA<PointT> pca;
    typename pcl::PointCloud<PointT>::Ptr PCACloud;
    PCACloud.reset(new pcl::PointCloud<PointT>());

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (labels[y][x] != -1)
                continue;

            seed_queue.push({x, y});
            boost::add_edge(label, label, g);

            int same_X[cols] = {0};
            same_X[0] = x;
            int line_size = 1;
            int same_Y = 0;
            while (!seed_queue.empty()) {
                auto current_seed = seed_queue.front();
                seed_queue.pop();
                int current_x = current_seed.first;
                int current_y = current_seed.second;
                if (labels[current_y][current_x] != -1)
                  continue;
                if(current_x == 0){
                  if(label == 1){
                    label--;
                  }
                  labels[current_y][current_x] = label;
                  PCACloud->push_back(input_->at(current_y * cols + lineIDX[current_x]));
                }
                else if(current_x == 15){
                  labels[current_y][current_x] = label;
                  PCACloud->push_back(input_->at(current_y * cols + lineIDX[current_x]));
                }
                else{
                  for(int t=0;t < cols;t++){
                    if(current_x == same_X[t]){
                      line_size = t+1;
                    }
                  }
                  same_Y = current_y;
                  labels[current_y][current_x] = label;
                  PCACloud->push_back(input_->at(current_y * cols + lineIDX[current_x]));
                }
                

                if(current_y*cols+current_x > cols*rows){
                  std::cout << "4???" << y*cols+x << std::endl;
                }

                float ch0 = calculateHorizontalCurvature(current_x, current_y);
                float cv0 = calculateVerticalCurvature(current_x, current_y);

                if (ch0 > T) {

                    addNeighborsToQueue(current_x, current_y+1, seed_queue);
                    addNeighborsToQueue(current_x, current_y-1, seed_queue);
                }
                if (cv0 > Y) {

                    addNeighborsToQueue(current_x+1, current_y, seed_queue);
                    addNeighborsToQueue(current_x-1, current_y, seed_queue);
                }
                // std::cout << "queue.size:" << seed_queue.size() << std::endl;
              }
            if(PCACloud->points.size() > 10){
              pca.setInputCloud(PCACloud);
              Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
              Eigen::Vector3f Max_vector = eigen_vectors.col(2); 
              Eigen::Vector4f pcaCentroidsource;
	            pcl::compute3DCentroid(*PCACloud, pcaCentroidsource);
              Eigen::Vector3f Centroid(pcaCentroidsource[0], pcaCentroidsource[1], pcaCentroidsource[2]);

              // std::cout << "==" << PCACloud->points[PCACloud->points.size()/2].x << "," << PCACloud->points[PCACloud->points.size()/2].y << "," << PCACloud->points[PCACloud->points.size()/2].z << std::endl;
              // std::cout << "eigen0" << eigen_vectors.col(0) << std::endl;
              // std::cout << "eigen1" << eigen_vectors.col(1) << std::endl;
              // std::cout << "eigen2" << eigen_vectors.col(2) << std::endl;
              // std::cout << "++++++++++++++++++++++++++++++" << label << std::endl;
              
              Aplane[label] = Max_vector[0];
              Bplane[label] = Max_vector[1];
              Cplane[label] = Max_vector[2];
              Dplane[label] = line_size; 
              Eplane[label] = same_X[0];
              Fplane[label] = same_Y;
              Fplane[label] = PCACloud->points[PCACloud->points.size()/2].z;
    
              Eigen::Vector3f Centroid1(PCACloud->points[PCACloud->points.size()/2].x, PCACloud->points[PCACloud->points.size()/2].y, PCACloud->points[PCACloud->points.size()/2].z);
              Gplane[label] = Centroid1.dot(Max_vector); 
              if(std::isnan(Max_vector[0])){
                get_plane_ransac(*PCACloud, Aplane[label], Bplane[label], Cplane[label], Gplane[label]);

                // std::cout << "A:" << Aplane[label]<< "B:" << Bplane[label]<< "C:" << Cplane[label]<< "G:" << Gplane[label] << std::endl;

                // std::cout << "++++++++++++++++" << label << std::endl;
              }
            }else{
              Aplane[label] = -1;
              Bplane[label] = -1;
              Cplane[label] = -1;
              Dplane[label] = -1;  
              Eplane[label] = -1;
              Fplane[label] = -1;
              Gplane[label] = -1;
            }
            

            PCACloud->clear();
            
            label++;
            // std::cout << "label:" << label << std::endl;
        }
    }

    // std::cout << "LABEL" << label << std::endl;
    // std::cout << "Aplane" << Aplane[5] << std::endl;
    // std::cout << "Bplane" << Bplane[5] << std::endl;
    // std::cout << "Cplane" << Cplane[5] << std::endl;
    // std::cout << "Dplane" << Dplane[5] << std::endl;

    Eigen::Vector3f vector_1(0.188761,0.0307982,-0.98154);
    Eigen::Vector3f vector_2(-0.00644889,0.0287161,0.999567);
    // std::cout << "angle:" << calRotation(vector_1,vector_2) << std::endl;
    
    // int label_s = 0;
    for(int i=0;i <= label;i++){

      if(Aplane[i] != -1){
           if(Fplane[i] < (-0.45) && Fplane[i] != -1){
                // std::cout << "A:" << Aplane[i] << std::endl;
                // std::cout << "B:" << Bplane[i] << std::endl;
                // std::cout << "C:" << Cplane[i] << std::endl;
                // std::cout << "------------"<< Gplane[i] <<"-------------" << i << "," << Fplane[i] <<std::endl;
            // boost::add_edge(label+2, i, g);
            }else{
            // boost::add_edge(label+1, i, g);
            }
            
        Eigen::Vector3f vector_v(Aplane[i],Bplane[i],Cplane[i]);
        for(int t=0;t <= label;t++){
          if(Aplane[t] != -1){
            Eigen::Vector3f vector_u(Aplane[t],Bplane[t],Cplane[t]);
            if(abs(calRotation(vector_v,vector_u)) < fusion_angle && abs(abs(Gplane[t]) - abs(Gplane[i])) < fusion_dis){
              boost::add_edge(t, i, g);
            }
          }
        }
      }
    }

    // std::cout << "label_s" << label_s << std::endl; 

 
    
    std::vector<size_t> label_map(boost::num_vertices(g));
    auto num_components = boost::connected_components(g, label_map.data());
    std::cout << "num_components:" << num_components << std::endl;
    clusters.resize(num_components);
    

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int index = y * cols + lineIDX[x];
            auto label = labels[y][x];
            auto new_label = label_map.at(label);
            clusters.at(new_label).indices.push_back(index);
        }
    }

    // Remove small and large clusters.

    auto read = clusters.begin();
    auto write = clusters.begin();
    for (; read != clusters.end(); ++read) {
      if (read->indices.size() >= 10 &&
          read->indices.size() <= max_cluster_size_) {  // min_cluster_size_    max_cluster_size_
        if (read != write) {
          *write = std::move(*read);
        }
        ++write;
      }
    }
    clusters.resize(std::distance(clusters.begin(), write));


    // pcl::PointCloud<pcl::PointXYZI>::Ptr I_pointcloud(
    //   new pcl::PointCloud<pcl::PointXYZI>);
    // auto re_label = 0;
    // for (const auto& cluster : clusters) {
    //   I_pointcloud->clear();
    //   for (auto index : cluster.indices) {
    //     auto& point = input_->at(index);
    //     // point.intensity = 0.0;
    //     pcl::PointXYZI i_point;
    //     i_point.x = point.x;
    //     i_point.y = point.y;
    //     i_point.z = point.z;
    //     i_point.intensity = 0.0;
    //     I_pointcloud->push_back(std::move(i_point));
    //   }

    //   get_plane_ransac(*I_pointcloud, Aplane[re_label], Bplane[re_label], Cplane[re_label], Dplane[re_label]);
    //   re_label++;
    // }

    // // auto read_1 = clusters.begin();
    // // auto write_1= clusters.begin();
    // // auto vecW_R[re_label];
    // // for(int i=0;i < re_label;i++){ 
    // //     vecW_R[i] = write_1;
    // //     ++write_1;
    // // }
    // // write_1 = clusters.begin();
    // // for(int i=0;i < re_label;i++){  
    // //     Eigen::Vector3f vector_v(Aplane[i],Bplane[i],Cplane[i]);
    // //     for(int t=i+1;t < re_label;t++){
    // //         read_1 = write_1;
    // //         ++read_1;
    // //         Eigen::Vector3f vector_u(Aplane[t],Bplane[t],Cplane[t]);
    // //         if(abs(calRotation(vector_v,vector_u)) < fusion_angle && abs(Dplane[t] - Dplane[i]) < fusion_dis){
    // //             if(vecW_R[t] == read_1){
    // //                 vecW_R[t] = write_1;
    // //             }
    // //             for (const auto& index : read_1->indices) {  
    // //                 vecW_R[t]->indices.push_back(index);  
    // //             }
    // //         }
    // //         ++read_1;
            
    // //     }
    // //     ++write_1;
    // // }

    // // int fusion[clusters.size()];
    // for (size_t i = 0; i < clusters.size(); ++i) {
    //     fusion[i] = i;  
    // }
    // std::vector<bool> fused(clusters.size(), false); 
    // for (size_t i = 0; i < clusters.size(); ++i) {
    //     Eigen::Vector3f vector_v(Aplane[i],Bplane[i],Cplane[i]);
    //     for (size_t j = i + 1; j < clusters.size(); ++j) {  
    //         if (!fused[j]) {  
    //             Eigen::Vector3f vector_u(Aplane[j],Bplane[j],Cplane[j]);

    //             if (abs(calRotation(vector_v,vector_u)) < 10 && abs(abs(Dplane[j]) - abs(Dplane[i])) < 0.2) {  
    //                 if(!fused[i]){
    //                     // clusters[i].indices.insert(clusters[i].indices.end(), clusters[j].indices.begin(), clusters[j].indices.end());
    //                     // clusters.erase(clusters.begin() + j);
    //                     // --j;

    //                     // for (const auto& idx : clusters[j].indices) {  
    //                     //     clusters[i].indices.push_back(idx);  
    //                     // } 
    //                     fusion[j] = i; 
    //                 }else{
    //                     // 
    //                     // clusters[fusion[i]].indices.insert(clusters[fusion[i]].indices.end(), clusters[j].indices.begin(), clusters[j].indices.end()); 
    //                     // clusters.erase(clusters.begin() + j);
    //                     // --j;

    //                     // for (const auto& idx : clusters[j].indices) {  
    //                     //     clusters[fusion[i]].indices.push_back(idx);  
    //                     // }  
    //                     fusion[j] = fusion[i];
    //                 }
                    
    //                 fused[j] = true; 
                    
    //             }  
    //         }  
    //     }  
    //     //fused[i] = true; 
    // }  

    // for (size_t i = 0; i < clusters.size(); ++i) {
    //     std::cout << "i=" << i << "," << fused[i] << ":" << fusion[i] << std::endl;
    // }
    
      
    // // 
    // // clusters.erase(std::remove_if(clusters.begin(), clusters.end(),  
    // //     [&fused](const pcl::PointIndices& cluster) { return fused[std::distance(clusters.begin(), std::addressof(cluster))]; }),  
    // //     clusters.end());




    deinitCompute();
  }

private:
  double cluster_tolerance_;
  size_t max_cluster_size_;
  size_t min_cluster_size_;
  double quality_;
  KdTreePtr tree_;
};
