#include "imi_Device.h"
#include <boost/thread/thread.hpp>
#include <stdio.h>
#include <string>
#include "ImiProperties.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/console.h>

namespace imi_wrapper
{
	static const double imi_factor = 1000.0;
	static const double imi_cx = 316.2;
	static const double imi_cy = 235.7;
	static const double imi_fx = 565.0;
	static const double imi_fy = 566.3;

	ImiDevice::ImiDevice() :
		pDeviceAttr(NULL),
		pImiDevice(NULL),
		depthHandle(NULL),
		colorHandle(NULL),
		camHandle(NULL),
		isDeviceValid(false),
		color_video_started_(0),
		depth_video_started_(0),
		uvc_video_started_(0),
        m_cx(imi_cx),
        m_cy(imi_cy),
        m_fx(imi_fx),
        m_fy(imi_fy),
		depthCallback(NULL),
		colorCallback(NULL),
        cloudCallback(NULL),
		uvcCallback(NULL)
	{
            
	}

	ImiDevice::~ImiDevice()
	{
		closeDevice();
	}

	int ImiDevice::openDevice()
	{
        //boost::lock_guard<boost::mutex> lock(device_mutex_);
		//1.imiInitialize()
		if (0 != imiInitialize(NULL))
		{
			ROS_WARN("ImiNect Init Failed!\n");
			return -1;
		}
		ROS_WARN("ImiNect Init Success.\n");

		uint32_t deviceCount = 0;
		imiGetDeviceList(&pDeviceAttr, &deviceCount);
		if (deviceCount <= 0 || NULL == pDeviceAttr)
		{
			ROS_WARN("Get No Connected Imidevice!\n");
			return -1;
		}
		ROS_WARN("Get %d Connected Imidevice.\n", deviceCount);

		//3.imiOpenDevice()
		if (0 != imiOpenDevice(pDeviceAttr[0].uri, &pImiDevice, 0))
		{
			ROS_WARN("Open Imidevice Failed!\n");
			return -1;
		}
		ROS_INFO("Imidevice Opened.\n");


        float cameraParams[4];
        uint32_t nLen = sizeof(float)*4;
        int32_t ret = imiGetDeviceProperty(pImiDevice, IMI_PROPERTY_COLOR_INTRINSIC_PARAMS, (void*)cameraParams, &nLen);
        if (ret == 0)
        {
        	m_fx = cameraParams[0];
            m_fy = cameraParams[1];
			m_cx = cameraParams[2];
            m_cy = cameraParams[3];
        }

        ROS_INFO("****** fx=%f  fy=%f  cx=%f  cy=%f\n", m_fx, m_fy, m_cx, m_cy);

		if (0 != imiCamOpen(&camHandle))
		{
			ROS_INFO("imiCamOpen Failed!\n");
            return -1;
		}
		
	    ROS_INFO("imiCamOpen Success.\n");

		isDeviceValid = true;

		boost::thread depthThread(readFrame, this);

        boost::thread depthThreadUVC(readUVCFrame, this);

		return 0;
	}

    void ImiDevice::convertToCloudPoint(ImiImageFrame* pFrame)
	{
		boost::shared_ptr<sensor_msgs::PointCloud2> cloudPtr(new sensor_msgs::PointCloud2);
		cloudPtr->width = pFrame->height * pFrame->width;
		cloudPtr->height = 1;

        cloudPtr->header.frame_id = "IMI_CloudPoints";
        
        sensor_msgs::PointCloud2Modifier modifier(*cloudPtr);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32);

		modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloudPtr, "x");
  		sensor_msgs::PointCloud2Iterator<float> iter_y(*cloudPtr, "y");
  		sensor_msgs::PointCloud2Iterator<float> iter_z(*cloudPtr, "z");

		uint16_t* pData = (uint16_t *)pFrame->pData;
		for (int j=0; j<pFrame->height; j++)
        {
        	for (int i=0; i<pFrame->width; i++)
        	{
				uint16_t dp = pData[j*pFrame->width + i];
		        if (dp > 0)
				{
                	double pz = dp/imi_factor;
                	double py = (j - m_cy)*pz/m_fy;
                	double px = (i - m_cx)*pz/m_fx;

                	*iter_x = px;
                	*iter_y = py;
					*iter_z = pz;
				}
				else
				{
					*iter_x = 0;
		            *iter_y = 0;
					*iter_z = 0;
				}
				++iter_x;
				++iter_y;
				++iter_z;
        	}
        }

        cloudCallback(cloudPtr);
	}

	int ImiDevice::readFrame(void* lParam)
	{
		ImiDevice* pIMi = (ImiDevice *)lParam;
		
		while (pIMi->isDeviceValid)
		{
			if (!(pIMi->depth_video_started_>0 || pIMi->color_video_started_>0))
			{
                                //ROS_WARN("wait for subscribers");
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				continue;
			}

            boost::lock_guard<boost::mutex> lock(pIMi->device_mutex_);

            int32_t g_streamNum = 0;
            ImiStreamHandle g_streams[2];

            if (pIMi->depth_video_started_)
            {
                g_streams[g_streamNum++] = pIMi->depthHandle;
            }

            if (pIMi->color_video_started_)
            {
                g_streams[g_streamNum++] = pIMi->colorHandle;
            }


			int32_t avStreamIndex;
			if (0 != imiWaitForStreams(g_streams, g_streamNum, &avStreamIndex, 100))
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(50));
				continue;
			}

			if ((avStreamIndex < 0) || ((uint32_t)avStreamIndex >= g_streamNum))
			{
				ROS_WARN("imiWaitForStream returns 0, but channel index abnormal: %d\n", avStreamIndex);
				continue;
			}

			ImiImageFrame* pFrame = NULL;
			if (0 != imiReadNextFrame(g_streams[avStreamIndex], &pFrame, 0))
			{
				ROS_WARN("imiReadNextFrame Failed, channel index : %d\n", avStreamIndex);
				continue;
			}

			sensor_msgs::ImagePtr image(new sensor_msgs::Image);

			if (pIMi->depthCallback!=NULL || (pIMi->colorCallback!=NULL && pFrame->pixelFormat==IMI_PIXEL_FORMAT_IMAGE_RGB24))
			{
				

				uint64_t device_time = pFrame->timeStamp;

				double device_time_in_sec = static_cast<double>(device_time) / 1000000.0;

				image->header.stamp.fromSec(device_time_in_sec);

				image->width = pFrame->width;
				image->height = pFrame->height;

				std::size_t data_size = pFrame->size;

				image->data.resize(data_size);
				memcpy(&image->data[0], pFrame->pData, data_size);

				image->is_bigendian = 0;
           	}


			if (pFrame->pixelFormat==IMI_PIXEL_FORMAT_IMAGE_RGB24 && pIMi->colorCallback!=NULL)
			{
                image->header.frame_id = "IMI_RGB_Image";
				image->encoding = sensor_msgs::image_encodings::RGB8;
				image->step = sizeof(unsigned char) * 3 * image->width;

				pIMi->colorCallback(image);
			}

			if (pFrame->pixelFormat == IMI_PIXEL_FORMAT_DEP_16BIT)
			{
                                
				if (pIMi->depthCallback != NULL)
				{
					image->header.frame_id = "IMI_DEPTH_Image";
					image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
					image->step = sizeof(unsigned char) * 2 * image->width;
					pIMi->depthCallback(image);
				}

				if (pIMi->cloudCallback != NULL)
				{
					pIMi->convertToCloudPoint(pFrame);
				}
			}

			imiReleaseFrame(&pFrame);
		}
	}

	int ImiDevice::readUVCFrame(void* lParam)
	{
		ImiDevice* pIMi = (ImiDevice *)lParam;
		
		while (pIMi->isDeviceValid)
		{
			if (!(pIMi->uvc_video_started_>0))
			{
                //ROS_WARN("wait for uvc subscribers");
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				continue;
			}

            boost::lock_guard<boost::mutex> lock(pIMi->uvc_mutex_);

            ImiCameraFrame* uvcFrame = NULL;
    		int ret = imiCamReadNextFrame(pIMi->camHandle, &uvcFrame, 40);
			if (ret!=0 || uvcFrame==NULL)
			{
				//ROS_WARN("imiCamReadNextFrame Failed\n");
				continue;
			}
			
			
			sensor_msgs::ImagePtr image(new sensor_msgs::Image);
                   

			uint64_t device_time = uvcFrame->timeStamp;

			double device_time_in_sec = static_cast<double>(device_time) / 1000000.0;

			image->header.stamp.fromSec(device_time_in_sec);



			image->width = uvcFrame->width;
			image->height = uvcFrame->height;

			std::size_t data_size = uvcFrame->size;

			image->data.resize(data_size);
			memcpy(&image->data[0], uvcFrame->pData, data_size);

			image->is_bigendian = 0;

			
            image->header.frame_id = "IMI_UVC_Image";
			image->encoding = sensor_msgs::image_encodings::RGB8;
			image->step = sizeof(unsigned char) * 3 * image->width;

            if (pIMi->uvcCallback != NULL)
			{
				pIMi->uvcCallback(image);
			}
			

			imiCamReleaseFrame(&uvcFrame);
		}
	}

	int ImiDevice::closeDevice()
	{	
        //boost::lock_guard<boost::mutex> lock(device_mutex_);
		isDeviceValid = false;

		stopAllStreams();

		if (NULL != pImiDevice)
		{
			imiCloseDevice(pImiDevice);
			pImiDevice = NULL;
		}

		if (NULL != pDeviceAttr)
		{
			imiReleaseDeviceList(&pDeviceAttr);
			pDeviceAttr = NULL;
		}

		imiDestroy();

        if (NULL != camHandle)
		{
			imiCamClose(camHandle);
			camHandle = NULL;
		}
	}

    sensor_msgs::CameraInfoPtr ImiDevice::getDefaultCameraInfo(int width, int height)
	{
		sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

	    info->width  = width;
		info->height = height;

		// distortion model and params
		info->D.resize(5, 0.0);
		info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

		// camera Params
		info->K.assign(0.0);
		info->K[0] = m_fx;
		info->K[4] = m_fy;
		info->K[2] = m_cx;
		info->K[5] = m_cy;
		info->K[8] = 1.0;

		// Rectification matrix (stereo cameras only)
		info->R.assign(0.0);
		info->R[0] = info->R[4] = info->R[8] = 1.0;

		// Projection/camera matrix
		info->P.assign(0.0);
		info->P[0]  = m_fx;
		info->P[5]  = m_fy; 
		info->P[2]  = m_cx; 
		info->P[6]  = m_cy;
		info->P[10] = 1.0;

		return info;
	}

	const std::string ImiDevice::getUri() const
	{
		if (pDeviceAttr != NULL)
		{
			return std::string(pDeviceAttr[0].uri);
		}

		return "";
	}

	uint16_t ImiDevice::getUsbVendorId() const
	{
		if (pDeviceAttr != NULL)
		{
			return pDeviceAttr[0].vendorId;
		}

		return 0;
	}

	uint16_t ImiDevice::getUsbProductId() const
	{
		if (pDeviceAttr != NULL)
		{
			return pDeviceAttr[0].productId;
		}

		return 0;
	}

	bool ImiDevice::isValid() const
	{
		return isDeviceValid;
	}

	bool ImiDevice::hasColorSensor() const
	{
		return true;
	}

	bool ImiDevice::hasUVCSensor() const
	{
		return true;
	}

	bool ImiDevice::hasDepthSensor() const
	{
		return true;
	}

	void ImiDevice::setColorFrameCallback(FrameCallbackFunction callback)
	{
        boost::lock_guard<boost::mutex> lock(device_mutex_);
		colorCallback = callback;
	}

	void ImiDevice::setUVCFrameCallback(FrameCallbackFunction callback)
	{
        boost::lock_guard<boost::mutex> lock(uvc_mutex_);
		uvcCallback = callback;
	}

	void ImiDevice::setDepthFrameCallback(FrameCallbackFunction callback)
	{
        boost::lock_guard<boost::mutex> lock(device_mutex_);
		depthCallback = callback;
	}

    void ImiDevice::setCloudPointCallback(CloudCallbackFunction callback)
    {
        boost::lock_guard<boost::mutex> lock(device_mutex_);
    	cloudCallback = callback;
    }

	int ImiDevice::startColorStream(ImiFrameMode colorframeMode)
	{
        boost::lock_guard<boost::mutex> lock(device_mutex_);
        color_video_started_++;

        if (color_video_started_ > 1)
        {
			printf("already Open Color Stream Success.\n");
			return 0;                
        }

		imiSetFrameMode(pImiDevice, IMI_COLOR_FRAME, &colorframeMode);

		if (0 != imiOpenStream(pImiDevice, IMI_COLOR_FRAME, NULL, NULL, &colorHandle))
		{
			ROS_WARN("Open Color Stream Failed!\n");
			return -1;
		}

		printf("Open Color Stream Success.\n");

		return 0;
	}

    int ImiDevice::startUVCStream(ImiCameraFrameMode uvcframeMode)
	{
        boost::lock_guard<boost::mutex> lock(uvc_mutex_);
        uvc_video_started_++;

        if (uvc_video_started_ > 1)
        {
			ROS_WARN("already Open UVC Stream Success.\n");
			return 0;                
        }

		if (0 != imiCamStartStream(camHandle, &uvcframeMode))
		{
			ROS_WARN("imiCamStartStream Failed!\n");
			return -1;
		}

		printf("Open UVC Stream Success.\n");

		return 0;
	}

	int ImiDevice::startDepthStream(ImiFrameMode frameMode)
	{
        boost::lock_guard<boost::mutex> lock(device_mutex_);
        depth_video_started_++;
        if (depth_video_started_ > 1)
        {
			printf("already Open Depth Stream Success.\n");
			return 0;
        }

		imiSetFrameMode(pImiDevice, IMI_DEPTH_FRAME, &frameMode);

        if (frameMode.resolutionX==640 && frameMode.resolutionY==480)
        {
        	m_cx = imi_cx;
            m_cy = imi_cy;
            m_fx = imi_fx;
            m_fy = imi_fy;
        }
        else
        {
            m_cx = imi_cx/2;
            m_cy = imi_cy/2;
            m_fx = imi_fx/2;
            m_fy = imi_fy/2;
        }

		if (0 != imiOpenStream(pImiDevice, IMI_DEPTH_FRAME, NULL, NULL, &depthHandle))
		{
			ROS_WARN("Open Depth Stream Failed!\n");
			return -1;
		}

		printf("Open Depth Stream Success.\n");

		return 0;
	}

	void ImiDevice::stopAllStreams()
	{
        if (color_video_started_ > 0)
        {
			color_video_started_ = 1;
        }

		stopColorStream();
        if (depth_video_started_ > 0)
        {
			depth_video_started_ = 1;
        }

		stopDepthStream();

        if (uvc_video_started_ > 0)
        {
			uvc_video_started_ = 1;
        }

		stopUVCStream();
	}

	void ImiDevice::stopColorStream()
	{
        boost::lock_guard<boost::mutex> lock(device_mutex_);
		if (color_video_started_ == 0)
        {
			return;
        }

		color_video_started_--;

		if (NULL != colorHandle && color_video_started_==0)
		{
            color_video_started_ = 0;
			imiCloseStream(colorHandle);
			colorHandle = NULL;	
		}
	}

	void ImiDevice::stopUVCStream()
	{
        boost::lock_guard<boost::mutex> lock(uvc_mutex_);
		if (uvc_video_started_ == 0)
        {
			return;
        }

		uvc_video_started_--;

		if (NULL != camHandle && uvc_video_started_==0)
		{
            uvc_video_started_ = 0;
			imiCamStopStream(camHandle);	
		}
	}

	void ImiDevice::stopDepthStream()
	{
        boost::lock_guard<boost::mutex> lock(device_mutex_);
        if (depth_video_started_ == 0)
        {
			return;
        }

        depth_video_started_--;
		if (NULL != depthHandle && depth_video_started_==0)
		{
            depth_video_started_ = 0;
			imiCloseStream(depthHandle);
			depthHandle = NULL;
		}
	}

}

