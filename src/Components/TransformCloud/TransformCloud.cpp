/*!
 * \file
 * \brief
 * \author Mort
 */

#include <memory>
#include <string>

#include "TransformCloud.hpp"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "Common/Logger.hpp"
#include "EventHandler.hpp"
#include "DataStream.hpp"
#include "Types/Objects3D/Object3D.hpp"
#include "Types/HomogMatrix.hpp"
#include "Types/CameraInfo.hpp"
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/bind.hpp>
#include "Property.hpp"
#include <math.h>


namespace Processors {
namespace TransformCloud {

typedef pcl::PointCloud<PointXYZSIFT> c_sift;
typedef pcl::PointCloud<pcl::PointXYZRGB> c_rgb;

TransformCloud::TransformCloud(const std::string & name) :
		Base::Component(name),
		prop_x("offset.x", 0),
		prop_y("offset.y", 0),
		prop_z("offset.z", 0),
		prop_roll("offset.roll", 0),
		prop_pitch("offset.pitch", 0),
		prop_yaw("offset.yaw", 0)
		{
			registerProperty(prop_x);
			registerProperty(prop_y);
			registerProperty(prop_z);
			registerProperty(prop_roll);
			registerProperty(prop_pitch);
			registerProperty(prop_yaw);
}

TransformCloud::~TransformCloud() {
}

void TransformCloud::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);

	// Register handlers
	h_transformCloud.setup(boost::bind(&TransformCloud::transformCloud, this));
	registerHandler("transformCloud", &h_transformCloud);
	addDependency("transformCloud", &in_cloud_xyzrgb);
	addDependency("transformCloud", &in_cloud_xyzsift);

}

bool TransformCloud::onInit() {

	return true;
}

bool TransformCloud::onFinish() {
	return true;
}

bool TransformCloud::onStop() {
	return true;
}

bool TransformCloud::onStart() {
	return true;
}

void TransformCloud::transformCloud(){
	CLOG(LDEBUG)<<"in transformCloud" <<"\n";
	if(!in_cloud_xyzrgb.empty()&&!in_cloud_xyzsift.empty()){

		cv::Mat_<double> wsp = cv::Mat::zeros(4,1,1);

		CLOG(LDEBUG) << "in transformCloud() \n";
		cv::Mat_<double> outputMatrix = cv::Mat::zeros(4,4,1);;
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

		// Roll - rotation around the X (blue) axis.
		cv::Mat roll = (cv::Mat_<double>(4, 4) <<
		              1,          0,           0, 0,
		              0, cos(prop_roll), -sin(prop_roll), 0,
		              0, sin(prop_roll),  cos(prop_roll), 0,
		              0, 0, 0, 1 );


		// Pitch - rotation around the Y (green) axis.
		cv::Mat pitch = (cv::Mat_<double>(4, 4) <<
	            cos(prop_pitch), 0, sin(prop_pitch), 0,
	            0, 1, 0, 0,
		        -sin(prop_pitch),  0,	cos(prop_pitch), 0,
		        0, 0, 0, 1 );

		// Yaw - rotation around the Z (red) axis.
		cv::Mat yaw = (cv::Mat_<double>(4, 4) <<
	            cos(prop_yaw), -sin(prop_yaw), 0, 0,
		        sin(prop_yaw),  cos(prop_yaw), 0, 0,
	            0, 0, 1, 0,
		        0, 0, 0, 1 );

		// translation
		cv::Mat t = (cv::Mat_<double>(4, 4) <<
				            0, 0, 0, prop_x,
					        0, 0, 0, prop_y,
				            0, 0, 0, prop_z,
					        0, 0, 0, 0 );

		outputMatrix = t + yaw * pitch * roll;

		Types::HomogMatrix hm;

		stringstream ss;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				transform(i,j)= hm.elements[i][j] = outputMatrix.at<double>(i,j);

				ss << hm.elements[i][j] << "  ";
			}
			ss << '\n';
		}



		CLOG(LINFO) << "HomogMatrix:\n" << ss.str() << endl;

		c_rgb::Ptr cloud_xyzrgb_read = in_cloud_xyzrgb.read();
		c_sift::Ptr cloud_xyzsift_read = in_cloud_xyzsift.read();
		c_rgb::Ptr cloud_xyzrgb = c_rgb::Ptr( new c_rgb);
		c_sift::Ptr cloud_xyzsift = c_sift::Ptr(new c_sift);
		//CLOG(LINFO) << "PRZED" << cloud_xyzrgb->at(0).x ;

		pcl::transformPointCloud(*cloud_xyzrgb_read, *cloud_xyzrgb, transform);
		pcl::transformPointCloud(*cloud_xyzsift_read, *cloud_xyzsift, transform);
		//CLOG(LINFO) << "PO" << cloud_xyzrgb->at(0).x ;
		out_cloud_xyzrgb.write(cloud_xyzrgb);
		out_cloud_xyzsift.write(cloud_xyzsift);
	}
}


} //: namespace TransformCloud
} //: namespace Processors
