#ifndef POINTXYZDESCRIPTOR_HPP_
#define POINTXYZDESCRIPTOR_HPP_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
//namespace Types {

struct PointXYZDescriptor
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  cv::Mat descriptor; 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
 
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZDescriptor           // here we assume a XYZ + "test" (as fields)
                                   ,(float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   //(cv::Mat, descriptor, descriptor)
)


//} //: namespace Types

#endif /* POINTXYZDESCRIPTOR_HPP_ */
