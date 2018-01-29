#ifndef IMI_DRIVER_H
#define IMI_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <imi_camera/ImiConfig.h>

namespace imi_wrapper
{

    class ImiDevice;

	class ImiDriver
	{

        typedef imi_ros_cfg::ImiConfig Config;
        typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	public:
		ImiDriver(ros::NodeHandle &n, ros::NodeHandle &pnh);
		~ImiDriver();
    private:
		int initDevice();
		void newColorFrameCallback(sensor_msgs::ImagePtr image);
		void newUVCFrameCallback(sensor_msgs::ImagePtr image);
		void newDepthFrameCallback(sensor_msgs::ImagePtr image);
        void newCloudPointCallback(boost::shared_ptr<sensor_msgs::PointCloud2> cloudPtr);
		void colorConnectCb();
		void uvcConnectCb();
		void depthConnectCb();
        void couldCb();
		void advertiseROSTopics();
        int startColorStream();
        int startUVCStream();
        int startDepthStream();
        void configCb(Config &config, uint32_t level);
	private:
		ros::NodeHandle& nh_;
		ros::NodeHandle& pnh_;
		ImiDevice* pImiDevice;
		bool color_subscribers_;
		bool depth_subscribers_;
        bool cloud_subscribers_;
        bool uvc_subscribers_;
		// published topics
		image_transport::CameraPublisher pub_color_;
		image_transport::CameraPublisher pub_depth_;
        ros::Publisher pub_cloud_;
        image_transport::CameraPublisher pub_uvc_;
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;
        bool config_init;
        int depthMode;
        int colorMode;
	};
}

#endif
