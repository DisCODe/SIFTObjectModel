/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "SolveChessboardLoc.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SolveChessboardLoc {

SolveChessboardLoc::SolveChessboardLoc(const std::string & name):
		Base::Component(name)  {

}

SolveChessboardLoc::~SolveChessboardLoc() {
}

void SolveChessboardLoc::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers

}

bool SolveChessboardLoc::onInit() {

	return true;
}

bool SolveChessboardLoc::onFinish() {
	return true;
}

bool SolveChessboardLoc::onStop() {
	return true;
}

bool SolveChessboardLoc::onStart() {
	return true;
}

void SolveChessboardLoc::solveLocation(){
	/*
	black X: 23,3; Y:7,5
	white X: 11,3; Y:7,5
	*/

	std::vector<cv::Point2f> image_points = in_impoints.read();


}

} //: namespace SolveChessboardLoc
} //: namespace Processors
