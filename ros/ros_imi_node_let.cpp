#include "imi_camera/imi_driver.h"
#include <nodelet/nodelet.h>

namespace imi_camera
{

	class ImiDriverNodelet : public nodelet::Nodelet
	{
	public:
		ImiDriverNodelet() {};

		~ImiDriverNodelet() {}

	private:
		virtual void onInit()
		{
			lp.reset(new imi_wrapper::ImiDriver(getNodeHandle(), getPrivateNodeHandle()));
		};

		boost::shared_ptr<astra_wrapper::ImiDriver> lp;
	};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(imi_camera, ImiDriverNodelet, imi_camera::ImiDriverNodelet, nodelet::Nodelet);
