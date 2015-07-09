/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef REPROJECTIONERROR_HPP_
#define REPROJECTIONERROR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include "Types/HomogMatrix.hpp"

namespace Processors {
namespace ReprojectionError {

/*!
 * \class ReprojectionError
 * \brief ReprojectionError processor class.
 *
 * 
 */
class ReprojectionError: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ReprojectionError(const std::string & name = "ReprojectionError");

	/*!
	 * Destructor
	 */
	virtual ~ReprojectionError();

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
	Base::DataStreamIn<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > in_rototranslations;
	Base::DataStreamIn<Eigen::Matrix4f> in_location;
    Base::DataStreamIn<Types::HomogMatrix> in_location_hm;

	// Output data streams

	// Properties

	
	// Handlers
    void calculate_errors(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations, Eigen::Matrix4f location);
    void calculate_errors_hm();
    void calculate_errors_eigen();

};

} //: namespace ReprojectionError
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ReprojectionError", Processors::ReprojectionError::ReprojectionError)

#endif /* REPROJECTIONERROR_HPP_ */
