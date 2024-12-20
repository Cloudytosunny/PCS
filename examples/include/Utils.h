#ifndef UTILS_H
#define UTILS_H

#include <queue>

#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>



float calculateHorizontalCurvature(int x, int y);
float calculateVerticalCurvature(int x, int y);
float calRotation(Eigen::Vector3f vec1, Eigen::Vector3f vec2);

template <typename PointT>
void get_plane_ransac(const pcl::PointCloud<PointT> & src_cloud,double &a,double &b,double &c,double &d);
template <typename PointT>
double cal_angle(PointT p1,PointT p2,PointT p3);
template <typename PointT>
double average_spacing(const PointT& point0, const PointT& point1,
                       const PointT& point2, const PointT& point3,
                       const PointT& point4, const PointT& point5);
template <typename PointT>
double distance(const PointT& p1, const PointT& p2);

#endif // UTILS_H
