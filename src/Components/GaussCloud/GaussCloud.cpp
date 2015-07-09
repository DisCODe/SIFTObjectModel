/*!
 * \file
 * \brief
 * \author Mort
 */

#include <memory>
#include <string>

#include "GaussCloud.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <cstdlib>
#include <ctime>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//
#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#define ranf() ((float) rand() / (float) RAND_MAX)

namespace Processors {
namespace GaussCloud {

GaussCloud::GaussCloud(const std::string & name) :
		Base::Component(name)  {

}

GaussCloud::~GaussCloud() {
}

void GaussCloud::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);

	// Register handlers
	h_makeNoisyCloud.setup(boost::bind(&GaussCloud::makeNoisyCloud, this));
	registerHandler("makeNoisyCloud", &h_makeNoisyCloud);
	addDependency("makeNoisyCloud", &in_cloud_xyzrgb);
	addDependency("makeNoisyCloud", &in_cloud_xyzsift);
}

bool GaussCloud::onInit() {
	srand(time(0));
	return true;
}

bool GaussCloud::onFinish() {
	return true;
}

bool GaussCloud::onStop() {
	return true;
}

bool GaussCloud::onStart() {
	return true;
}

void GaussCloud::makeNoisyCloud(){
	CLOG(LDEBUG)<<"in makeNoisyCloud" <<"\n";
	if(!in_cloud_xyzrgb.empty()&&!in_cloud_xyzsift.empty()){
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();
		pcl::PointCloud <PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();

		std::vector<int> indices;
			cloud_xyzrgb->is_dense = false;
			pcl::removeNaNFromPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, indices);
		CLOG(LDEBUG) << "in makeNoisyCloud() \n";
		cv::Mat_<double> wsp = cv::Mat::zeros(4,1,1);

		float prop_x;
		float prop_y;
		float prop_z;
		cv::Mat t;

		for(int index=0; index< cloud_xyzrgb->size(); index++){
			cloud_xyzrgb->at(index).x+= GaussCloud::generateNumber(0, 0.001);
			cloud_xyzrgb->at(index).y+= GaussCloud::generateNumber(0, 0.001);
			cloud_xyzrgb->at(index).z+= GaussCloud::generateNumber(0, 0.001);

			cloud_xyzrgb->at(index).r+=GaussCloud::generateNumber(1, 10);
			cloud_xyzrgb->at(index).g+=GaussCloud::generateNumber(1, 10);
			cloud_xyzrgb->at(index).b+=GaussCloud::generateNumber(1, 10);

		}

		for(int index=0; index< cloud_xyzsift->size(); index++){
			cloud_xyzsift->at(index).x+= GaussCloud::generateNumber(0, 0.001);
			cloud_xyzsift->at(index).y+= GaussCloud::generateNumber(0, 0.001);
			cloud_xyzsift->at(index).z+= GaussCloud::generateNumber(0, 0.001);

		}

		out_cloud_xyzsift.write(cloud_xyzsift);
		out_cloud_xyzrgb.write(cloud_xyzrgb);
	}
}

float GaussCloud::generateNumber(float m, float s){
	   static int pass = 0;
	   static float y2;
	   float x1, x2, w, y1;

	   if (pass)
	   {
	      y1 = y2;
	   } else  {
	      do {
	         x1 = 2.0f * ranf () - 1.0f;
	         x2 = 2.0f * ranf () - 1.0f;
	         w = x1 * x1 + x2 * x2;
	      } while (w >= 1.0f);

	      w = (float)sqrt(-2.0 * log (w) / w);
	      y1 = x1 * w;
	      y2 = x2 * w;
	   }
	   pass = !pass;
	//   CLOG(LINFO) << "number randoM is: " << (y1 * s + (float) m) ;
	   return ( (y1 * s + (float) m));
}

} //: namespace GaussCloud
} //: namespace Processors
