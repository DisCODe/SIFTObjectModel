#ifndef SIFTOBJECTINSTANCE_HPP_
#define SIFTOBJECTINSTANCE_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointCloudObject.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointCloudNormalObject.hpp>

//namespace Types {

/*!
 * \class SIFTObjectInstance
 * \brief Instance of 3D object.
 * It consists of: object point cloud (empty as default), model SIFT cloud, instance SIFT cloud, SOM name.
 */
class SIFTObjectModel : public PointCloudObject
{
	public:
	/// Cloud of SIFT - features extracted from RGB image and transformed from image into Cartesian space.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;

	/// Name of the model.
	std::string model_id;

	/// Instance id.
	std::string instance_id;

	/// Reprojection error.
	std::double RPE;





};


//} //: namespace Types

#endif /* SIFTOBJECTINSTANCE_HPP_ */
