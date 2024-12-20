#include "Utils.h"

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

template <typename PointT>
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

template <typename PointT>
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

template <typename PointT>
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

template <typename PointT>
double distance(const PointT& p1, const PointT& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}