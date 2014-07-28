/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "DrawCSystem.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include "Property.hpp"

namespace Processors {
namespace DrawCSystem {

DrawCSystem::DrawCSystem(const std::string & name) :
		Base::Component(name){
}

DrawCSystem::~DrawCSystem() {
}

void DrawCSystem::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_rvec", &in_rvec);
	registerStream("in_tvec", &in_tvec);
	registerStream("in_camera_matrix", &in_camera_matrix);

	registerStream("out_csystem", &out_csystem);
	registerStream("out_impoints", &out_impoints);

	// Register handlers
	h_projectPoints.setup(this, &DrawCSystem::projectPoints);
	registerHandler("projectPoints", &h_projectPoints);
	addDependency("projectPoints", &in_tvec);
	addDependency("projectPoints", &in_rvec);
	addDependency("projectPoints", &in_camera_matrix);
}

bool DrawCSystem::onInit() {

	return true;
}

bool DrawCSystem::onFinish() {
	return true;
}

bool DrawCSystem::onStop() {
	return true;
}

bool DrawCSystem::onStart() {
	return true;
}

void DrawCSystem::projectPoints(){

	cv::Mat rvec = in_rvec.read();
	cv::Mat tvec = in_tvec.read();
	Types::CameraInfo camera_matrix = in_camera_matrix.read();

	vector<cv::Point3f> object_points;
	vector<cv::Point2f> image_points;

	object_points.push_back(cv::Point3f(0,0,0));
	object_points.push_back(cv::Point3f(0.1,0,0));
	object_points.push_back(cv::Point3f(0,0.1,0));
	object_points.push_back(cv::Point3f(0,0,0.1));

	cv::projectPoints(object_points, rvec, tvec, camera_matrix.cameraMatrix(), camera_matrix.distCoeffs(), image_points);

	Types::Line ax(image_points[0], image_points[1]);
	ax.setCol(cv::Scalar(255, 0, 0));
	Types::Line ay(image_points[0], image_points[2]);
	ay.setCol(cv::Scalar(0, 255, 0));
	Types::Line az(image_points[0], image_points[3]);
	az.setCol(cv::Scalar(0, 0, 255));

	Types::DrawableContainer ctr;
	ctr.add(ax.clone());
	ctr.add(ay.clone());
	ctr.add(az.clone());

	CLOG(LINFO)<<"Image Points: "<<image_points;

	out_csystem.write(ctr);
	out_impoints.write(image_points);
}


} //: namespace DrawCSystem
} //: namespace Processors
