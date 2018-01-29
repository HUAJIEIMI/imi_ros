#include "imi_Driver.h"
#include "imi_Device.h"
#include <unistd.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <sys/shm.h>  
#include <time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include "ImiDefines.h"
#include "ImiProperties.h"
#include <boost/thread/thread.hpp>

namespace imi_wrapper
{

	static const int64_t NUM_MILLISEC_PER_SEC = 1000;
	static const int64_t NUM_MICROSECS_PER_SEC = 1000000;
	static const int64_t NUM_NANNOSECS_PER_SEC = 1000000000;

	static const int64_t NUM_MICROSECS_PER_MILLESEC = NUM_MICROSECS_PER_SEC /
	    NUM_MILLISEC_PER_SEC;
	static const int64_t NUM_NANOSECS_PER_MILLESEC =  NUM_NANNOSECS_PER_SEC /
	    NUM_MILLISEC_PER_SEC;
	static const int64_t NUM_NANOSECS_PER_MICROSEC =  NUM_NANNOSECS_PER_SEC /
	    NUM_MICROSECS_PER_SEC;
	
	uint64_t imi_timeNanos()
	{
	    uint64_t ticks = 0;
	    struct timespec ts;
	    clock_gettime (CLOCK_MONOTONIC, &ts);
	    ticks = NUM_NANNOSECS_PER_SEC * static_cast<uint64_t> (ts.tv_sec) +
		static_cast<uint64_t> (ts.tv_nsec);

	    return ticks;
	}
	uint64_t imi_time()
	{
	    return static_cast<uint64_t> (imi_timeNanos() / NUM_NANOSECS_PER_MILLESEC);
	}

    uint64_t depthStartTime = 0;
    uint64_t depthCount = 0;

    uint64_t uvcStartTime = 0;
    uint64_t uvcCount = 0;

    ImiFrameMode supportFrameMode[] = {{IMI_PIXEL_FORMAT_DEP_16BIT, 640, 480, 16, 30}, 
                                       {IMI_PIXEL_FORMAT_DEP_16BIT, 320, 240, 16, 30}};

	ImiCameraFrameMode supportCameraFrameMode[] = {{CAMERA_PIXEL_FORMAT_RGB888, 640, 480, 30}, 
                                                   {CAMERA_PIXEL_FORMAT_RGB888, 960, 720, 30},
                                                   {CAMERA_PIXEL_FORMAT_RGB888, 1280, 720, 30},
                                                   {CAMERA_PIXEL_FORMAT_RGB888, 1920, 1080, 30}};

	ImiDriver::ImiDriver(ros::NodeHandle &n, ros::NodeHandle &pnh) : 
          nh_(n),
          pnh_(pnh),
          color_subscribers_(false),
          depth_subscribers_(false),
          cloud_subscribers_(false),
	      uvc_subscribers_(false),
          config_init(false),
          depthMode(0),
          colorMode(0)
	{
        reconfigure_server_.reset(new ReconfigureServer(pnh_));
        reconfigure_server_->setCallback(boost::bind(&ImiDriver::configCb, this, _1, _2));
        while (!config_init)
  	    {
            ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }

		pImiDevice = new ImiDevice();
        int ret = initDevice();
        if (ret == 0)
        {
            advertiseROSTopics();
        }
	}

	ImiDriver::~ImiDriver()
	{
		if (pImiDevice != NULL)
		{
			delete pImiDevice;
		}
	}

    void ImiDriver::configCb(Config &config, uint32_t level)
    {
        depthMode = config.depth_mode - 1;
        colorMode = config.color_mode - 1;
        printf("************ %d  %d\n", config.depth_mode, config.color_mode);
        config_init = true;
    }

	int ImiDriver::initDevice()
	{
		int ret = pImiDevice->openDevice();
		if (ret != 0)
		{
			pImiDevice->closeDevice();
		}

		return ret;
	}

    int ImiDriver::startDepthStream()
    {
        ImiFrameMode frameMode;
        frameMode.pixelFormat = supportFrameMode[depthMode].pixelFormat;
        frameMode.resolutionX = supportFrameMode[depthMode].resolutionX;
        frameMode.resolutionY = supportFrameMode[depthMode].resolutionY;
        frameMode.bitsPerPixel = supportFrameMode[depthMode].bitsPerPixel;
        frameMode.framerate = supportFrameMode[depthMode].framerate;
        pImiDevice->startDepthStream(frameMode);

    }

    int ImiDriver::startColorStream()
    {
        ImiFrameMode frameMode;
        frameMode.pixelFormat = IMI_PIXEL_FORMAT_IMAGE_RGB24;
        frameMode.resolutionX = 640;
        frameMode.resolutionY = 480;
        frameMode.bitsPerPixel = 16;
        frameMode.framerate = 30;
        pImiDevice->startColorStream(frameMode);
    }

	int ImiDriver::startUVCStream()
    {
        ImiCameraFrameMode frameMode;
        frameMode.pixelFormat = supportCameraFrameMode[colorMode].pixelFormat;
        frameMode.resolutionX = supportCameraFrameMode[colorMode].resolutionX;
        frameMode.resolutionY = supportCameraFrameMode[colorMode].resolutionY;
        frameMode.fps = supportCameraFrameMode[colorMode].fps;
        pImiDevice->startUVCStream(frameMode);
    }


	void ImiDriver::newColorFrameCallback(sensor_msgs::ImagePtr image)
	{
		if (color_subscribers_)
		{
            sensor_msgs::CameraInfoPtr info = pImiDevice->getDefaultCameraInfo(640, 480);
                        
			timespec timenow;
            clock_gettime(CLOCK_REALTIME, &timenow); 
		
			//info->header.stamp.sec = timenow.tv_sec;
                        //info->header.stamp.nsec = timenow.tv_nsec;
			//image->header.stamp.sec = info->header.stamp.sec;
			//image->header.stamp.nsec = info->header.stamp.nsec;
			
			image->header.stamp = ros::Time::now();
            info->header.stamp = image->header.stamp;
			//printf("******header.stamp.sec=%lld header.stamp.nsec=%lld\n", image->header.stamp.sec, image->header.stamp.nsec);

            info->header.frame_id = image->header.frame_id;
			pub_color_.publish(image, info);
		}
	}

	void ImiDriver::newUVCFrameCallback(sensor_msgs::ImagePtr image)
	{
		if (uvc_subscribers_)
		{
			uvcCount++;

            if(uvcStartTime == 0)
            {
				uvcStartTime = imi_time();
            }

            sensor_msgs::CameraInfoPtr info = pImiDevice->getDefaultCameraInfo(image->width, image->height);
                       
			timespec timenow;
            clock_gettime(CLOCK_REALTIME, &timenow); 
		
			//info->header.stamp.sec = timenow.tv_sec;
            //info->header.stamp.nsec = timenow.tv_nsec;
			//image->header.stamp.sec = info->header.stamp.sec;
			//image->header.stamp.nsec = info->header.stamp.nsec;
			image->header.stamp = ros::Time::now();
            info->header.stamp = image->header.stamp;
			//printf("******header.stamp.sec=%lld header.stamp.nsec=%lld\n", image->header.stamp.sec, image->header.stamp.nsec);

            info->header.frame_id = image->header.frame_id;
			pub_uvc_.publish(image, info);

			if (imi_time()-uvcStartTime >= 10000)
            {
                //printf("****** UVC FPS = %f\n", uvcCount/10.0f);
				uvcStartTime = imi_time();
                uvcCount = 0;
            }
		}
	}

	void ImiDriver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
	{

		if (depth_subscribers_)
		{
			depthCount++;

            if(depthStartTime == 0)
            {
				depthStartTime = imi_time();
            }
            sensor_msgs::CameraInfoPtr info = pImiDevice->getDefaultCameraInfo(image->width, image->height);
            
            timespec timenow;
            clock_gettime(CLOCK_REALTIME, &timenow); 

			//info->header.stamp.sec = timenow.tv_sec;
			            //info->header.stamp.nsec = timenow.tv_nsec;
			//image->header.stamp.sec = info->header.stamp.sec;
			//image->header.stamp.nsec = info->header.stamp.nsec;
            image->header.stamp = ros::Time::now();
            info->header.stamp = image->header.stamp;
            //printf("******header.stamp.sec=%lld header.stamp.nsec=%lld\n", image->header.stamp.sec, image->header.stamp.nsec);
            info->header.frame_id = image->header.frame_id;

			pub_depth_.publish(image, info);

            if (imi_time()-depthStartTime >= 10000)
            {
                //printf("****** Depth FPS = %f\n", depthCount/10.0f);
				depthStartTime = imi_time();
                depthCount = 0;
            }
		}
	}

        void ImiDriver::newCloudPointCallback(boost::shared_ptr<sensor_msgs::PointCloud2> cloudPtr)
        {
			if (cloud_subscribers_)
			{
	            pub_cloud_.publish(cloudPtr);
			}
        }


	void ImiDriver::colorConnectCb()
	{
		color_subscribers_ = pub_color_.getNumSubscribers() > 0;

		if (color_subscribers_)
		{
            pImiDevice->setColorFrameCallback(boost::bind(&ImiDriver::newColorFrameCallback, this, _1));
            startColorStream();
	        printf("****** color_subscribers_ in \n");
		}
		else
        {
            pImiDevice->stopColorStream();
            pImiDevice->setColorFrameCallback(NULL);
            printf("****** color_subscribers_ out \n");
        }
	}

	void ImiDriver::uvcConnectCb()
	{
		uvc_subscribers_ = pub_uvc_.getNumSubscribers() > 0;

		if (uvc_subscribers_)
		{
            pImiDevice->setUVCFrameCallback(boost::bind(&ImiDriver::newUVCFrameCallback, this, _1));
            startUVCStream();
	        printf("****** uvc_subscribers_ in \n");
		    uvcStartTime = 0;
            uvcCount = 0;
		}
		else
        {
            pImiDevice->stopUVCStream();
            pImiDevice->setUVCFrameCallback(NULL);
            printf("****** uvc_subscribers_ out \n");
        }
	}

	void ImiDriver::depthConnectCb()
	{
		depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;

		if (depth_subscribers_)
		{
            pImiDevice->setDepthFrameCallback(boost::bind(&ImiDriver::newDepthFrameCallback, this, _1));
            startDepthStream();
            printf("****** depth_subscribers_ in \n");
            depthStartTime = 0;
            depthCount = 0;
		}
        else
        {
    		pImiDevice->stopDepthStream();
            pImiDevice->setDepthFrameCallback(NULL);
            printf("****** depth_subscribers_ out \n");
        }
	}

    void ImiDriver::couldCb()
	{
		cloud_subscribers_ = pub_cloud_.getNumSubscribers() > 0;

		if (cloud_subscribers_)
		{
            pImiDevice->setCloudPointCallback(boost::bind(&ImiDriver::newCloudPointCallback, this, _1));
            startDepthStream();
            printf("****** cloud_subscribers_ in \n");
		}
        else
        {
            pImiDevice->stopDepthStream();
            pImiDevice->setCloudPointCallback(NULL);
            printf("****** cloud_subscribers_ out \n");
        }
	}


	void ImiDriver::advertiseROSTopics()
	{
		ros::NodeHandle color_nh(nh_, "rgb");
		image_transport::ImageTransport color_it(color_nh);

		ros::NodeHandle uvc_nh(nh_, "uvc");
		image_transport::ImageTransport uvc_it(uvc_nh);

		ros::NodeHandle depth_nh(nh_, "depth");
		image_transport::ImageTransport depth_it(depth_nh);

        ros::NodeHandle cloud_it(nh_, "cloud");

		if (pImiDevice->hasColorSensor())
		{
			image_transport::SubscriberStatusCallback itssc = boost::bind(&ImiDriver::colorConnectCb, this);
			ros::SubscriberStatusCallback rssc = boost::bind(&ImiDriver::colorConnectCb, this);
			pub_color_ = color_it.advertiseCamera("image", 1, itssc, itssc);
		}

		if (pImiDevice->hasUVCSensor())
		{
			image_transport::SubscriberStatusCallback itssc = boost::bind(&ImiDriver::uvcConnectCb, this);
			ros::SubscriberStatusCallback rssc = boost::bind(&ImiDriver::uvcConnectCb, this);
			pub_uvc_ = uvc_it.advertiseCamera("image", 1, itssc, itssc);
		}

		if (pImiDevice->hasDepthSensor())
		{
			image_transport::SubscriberStatusCallback itssc = boost::bind(&ImiDriver::depthConnectCb, this);
			ros::SubscriberStatusCallback rssc = boost::bind(&ImiDriver::depthConnectCb, this);
			pub_depth_ = depth_it.advertiseCamera("image", 1, itssc, itssc);

            ros::SubscriberStatusCallback itssc_cloud = boost::bind(&ImiDriver::couldCb, this);
			pub_cloud_ = cloud_it.advertise<sensor_msgs::PointCloud2>("image", 1, itssc_cloud, itssc_cloud);
		}
	}

}
