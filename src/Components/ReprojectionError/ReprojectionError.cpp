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
//	registerStream("in_location", &in_location);
    registerStream("in_location_hm", &in_location_hm);
	// Register handlers
//    registerHandler("calculate_errors_eigen", boost::bind(&ReprojectionError::calculate_errors_eigen, this));
//    addDependency("calculate_errors_eigen", &in_rototranslations);
//    addDependency("calculate_errors_eigen", &in_location);
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

//void ReprojectionError::calculate_errors_eigen() {
//    CLOG(LTRACE) << "ReprojectionError::calculate_errors_eigen";
//    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
//    Eigen::Matrix4d location = in_location.read();
//    calculate_errors(rototranslations, location);
//}

void ReprojectionError::calculate_errors_hm() {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors_hm";
    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
    Types::HomogMatrix location = in_location_hm.read();
//    Eigen::Matrix4d l =  Eigen::Matrix4d::Identity();
//    for(int i = 0; i < 4; i++)
//        for(int j = 0; j < 4; j++){
//            l(i,j) = location(i, j);
//        }
    calculate_errors(rototranslations, location);
}

void ReprojectionError::calculate_errors(std::vector<Types::HomogMatrix> rototranslations, Types::HomogMatrix location) {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors";
    CLOG(LDEBUG) << "Ground truth before set (3,3) 1:\n" << location ;
    location(3,3) = 1;
    CLOG(LDEBUG) << "Ground truth:\n" << location ;
    Types::HomogMatrix inv;
    inv.matrix() = location.matrix().inverse();
    CLOG(LDEBUG) << "Ground truth inversed:\n" << inv ;

    if(rototranslations.empty()){
        CLOG(LINFO)<< "ReprojectionError No hypotheses available" ;
    }
    for(int i = 0; i < rototranslations.size(); i++){
        CLOG(LDEBUG)<< "Object Translation before set (3,3) 1:" << i << endl << rototranslations[i]<<endl;
        rototranslations[i](3,3) = 1;
        CLOG(LDEBUG)<< "Object Translation " << i << endl << rototranslations[i]<<endl;
        Types::HomogMatrix dT;
        dT.matrix() = rototranslations[i].matrix() * inv.matrix();
        CLOG(LDEBUG)<< "dT" << endl << dT <<endl;

        double et = sqrt( pow(dT(0,3), 2) + pow(dT(1,3), 2) + pow(dT(2,3), 2));
        double er = acos((dT.matrix().trace()-1-1)/2);
        CLOG(LINFO)<< "ReprojectionError Hypothese " << i << " et "<< et << " er " << er<< " x " << dT(0,3) << " y " << dT(1,3) << " z " << dT(2,3) << endl;

    }
}



} //: namespace ReprojectionError
} //: namespace Processors
