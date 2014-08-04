/*!
 * \file
 * \brief 
 * \author Marta Lepicka
 */

#ifndef SOLVECHESSBOARDLOC_HPP_
#define SOLVECHESSBOARDLOC_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/HomogMatrix.hpp"
#include "Types/CameraInfo.hpp"
#include "Types/DrawableContainer.hpp"
#include "Types/Line.hpp"


namespace Processors {
namespace SolveChessboardLoc {

/*!
 * \class SolveChessboardLoc
 * \brief SolveChessboardLoc processor class.
 *
 * 
 */
class SolveChessboardLoc: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SolveChessboardLoc(const std::string & name = "SolveChessboardLoc");

	/*!
	 * Destructor
	 */
	virtual ~SolveChessboardLoc();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn <std::vector<cv::Point2f> > in_impoints;
	// Output data streams

	// Handlers

	// Properties

	
	// Handlers


	void solveLocation();
};

} //: namespace SolveChessboardLoc
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SolveChessboardLoc", Processors::SolveChessboardLoc::SolveChessboardLoc)

#endif /* SOLVECHESSBOARDLOC_HPP_ */
