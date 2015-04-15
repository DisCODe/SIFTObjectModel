/*!
 * \file CvGaussianBlur_Processor.hpp
 * \brief
 * \author mstefanc
 * \date 2010-07-05
 */

#ifndef CVGAUSSIANBLUR_PROCESSOR_HPP_
#define CVGAUSSIANBLUR_PROCESSOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Types/stream_OpenCV.hpp"

/**
 * \defgroup CvGaussianBlur CvGaussianBlur
 * \ingroup Processors
 *
 * Gaussian blur filter, based on OpenCV.
 *
 *
 *
 * \par Data streams:
 *
 * \streamin{in_img,cv::Mat}
 * Input image
 * \streamout{out_img,cv::Mat}
 * Output image
 *
 *
 * \par Events:
 *
 * \event{newImage}
 * New image is ready
 *
 *
 * \par Event handlers:
 *
 * \handler{onNewImage}
 * New image arrived
 *
 *
 * \par Properties:
 *
 * \prop{kernel,cv::Size2i,7 7}
 * The Gaussian kernel size; kernel width and height can differ, but they both must be positive and odd.
 * Or, they can be zero’s, then they are computed from sigma*
 * \prop{sigmax,double,0.0}
 * \prop{sigmay,double,0.0}
 * The Gaussian kernel standard deviations in X and Y direction. If sigmay is zero, it is set to be equal to sigmax.
 * If they are both zeros, they are computed from kernel width and height, respectively.
 *
 *
 * \see http://opencv.willowgarage.com/documentation/cpp/image_filtering.html?highlight=gaus#GaussianBlur
 * @{
 *
 * @}
 */

namespace Processors {
namespace CvGaussianBlur {

using namespace cv;

/*!
 * \brief CvThreshold properties
 */
/*
struct BlurProps: public Base::Props
{

	!
	 * \copydoc Base::Props::load

	void load(const ptree & pt)
	{
		kernel = pt.get("kernel", cv::Size2i(71,71));

		sigmax = pt.get("sigmax", 0.0);
		sigmay = pt.get("sigmay", 0.0);
	}

	!
	 * \copydoc Base::Props::save

	void save(ptree & pt)
	{
		pt.put("kernel", kernel);

		pt.put("sigmax", sigmax);
		pt.put("sigmay", sigmay);
	}

};
*/

/*!
 * \class CvGaussianBlur_Processor
 * \brief Example processor class.
 */
class CvGaussianBlur_Processor: public Base::Component
{
public:
	/*!
	 * Constructor.
	 */
	CvGaussianBlur_Processor(const std::string & name = "");

	/*!
	 * Destructor
	 */
	virtual ~CvGaussianBlur_Processor();

	/*!
	 * Prepares communication interface.
	 */
	virtual void prepareInterface();

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
	 * Retrieves data from device.
	 */
	bool onStep();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	/*!
	 * Event handler function.
	 */
	void onNewImage();

	/// Event handler.
	Base::EventHandler <CvGaussianBlur_Processor> h_onNewImage;

	/// Input data stream
	Base::DataStreamIn <Mat> in_img;

	/// Output data stream - processed image
	Base::DataStreamOut <Mat> out_img;

	/// Gaussian kernel standard deviation in X direction.
	Base::Property<double> sigmax;

	/// Gaussian kernel standard deviation in Y direction.
	Base::Property<double> sigmay;

	/// Size of the kernel - width.
	Base::Property<int> kernel_width;

	/// Size of the kernel - height.
	Base::Property<int> kernel_height;

};

}//: namespace CvGaussianBlur
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("CvGaussianBlur", Processors::CvGaussianBlur::CvGaussianBlur_Processor)

#endif /* CVGAUSSIANBLUR_PROCESSOR_HPP_ */

