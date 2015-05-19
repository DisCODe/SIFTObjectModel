/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "ReprojectionError.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ReprojectionError {

ReprojectionError::ReprojectionError(const std::string & name) :
		Base::Component(name)  {

}

ReprojectionError::~ReprojectionError() {
}

void ReprojectionError::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_rototranslations", &in_rototranslations);
	registerStream("in_location", &in_location);
    registerStream("in_location_hm", &in_location_hm);
	// Register handlers
    registerHandler("calculate_errors_eigen", boost::bind(&ReprojectionError::calculate_errors_eigen, this));
    addDependency("calculate_errors_eigen", &in_rototranslations);
    addDependency("calculate_errors_eigen", &in_location);
    registerHandler("calculate_errors_hm", boost::bind(&ReprojectionError::calculate_errors_hm, this));
    addDependency("calculate_errors_hm", &in_rototranslations);
    addDependency("calculate_errors_hm", &in_location_hm);

}

bool ReprojectionError::onInit() {

	return true;
}

bool ReprojectionError::onFinish() {
	return true;
}

bool ReprojectionError::onStop() {
	return true;
}

bool ReprojectionError::onStart() {
	return true;
}

void ReprojectionError::calculate_errors_eigen() {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors_eigen";
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = in_rototranslations.read();
    Eigen::Matrix4f location = in_location.read();
    calculate_errors(rototranslations, location);
}

void ReprojectionError::calculate_errors_hm() {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors_hm";
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = in_rototranslations.read();
    Types::HomogMatrix location = in_location_hm.read();
    calculate_errors(rototranslations, location.getElements());
}

void ReprojectionError::calculate_errors(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations, Eigen::Matrix4f location) {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors";
    location(3,3) = 1;
    CLOG(LDEBUG)<< endl << location ;
    if(rototranslations.empty()){
        CLOG(LINFO)<< "ReprojectionError No hypotheses available" ;
    }
    for(int i = 0; i < rototranslations.size(); i++){
        rototranslations[i](3,3) = 1;

        CLOG(LDEBUG)<< "Translation " << i << endl << rototranslations[i]<<endl;
        Eigen::Matrix4f dT = rototranslations[i] * location.inverse();
        CLOG(LDEBUG)<< "dT" << endl << dT <<endl;
        float et = sqrt( pow(dT(0,3), 2) + pow(dT(1,3), 2) + pow(dT(2,3), 2));
        float er = acos((dT.trace()-1-1)/2);
        CLOG(LINFO)<< "ReprojectionError Hypothese " << i << " et "<< et << " er " << er<< endl;

    }
}



} //: namespace ReprojectionError
} //: namespace Processors
