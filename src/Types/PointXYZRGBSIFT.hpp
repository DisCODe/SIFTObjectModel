#ifndef POINTXYZRGBSIFT_HPP_
#define POINTXYZRGBSIFT_HPP_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
//namespace Types {

struct PointXYZRGBSIFT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  cv::Mat descriptor; 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBSIFT           // here we assume a XYZ + "test" (as fields)
                                   ,(float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   //(cv::Mat, descriptor, descriptor)
)


//} //: namespace Types

#endif /* POINTXYZRGBSIFT_HPP_ */
