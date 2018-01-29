#pragma once

#ifndef IMI_DEVICE_H
#define IMI_DEVICE_H

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "ImiNect.h"
#include "ImiCamera.h"
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>



namespace imi_wrapper
{
	typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;
    typedef boost::function<void(boost::shared_ptr<sensor_msgs::PointCloud2> cloudPtr)> CloudCallbackFunction;

	class ImiDevice
	{
	public:
		ImiDevice();
		virtual ~ImiDevice();

		const std::string getUri() const;
		uint16_t getUsbVendorId() const;
		uint16_t getUsbProductId() const;

		bool isValid() const;

		bool hasColorSensor() const;
		bool hasUVCSensor() const;
		bool hasDepthSensor() const;

		int startColorStream(ImiFrameMode colorFrameMode);
		int startUVCStream(ImiCameraFrameMode uvcframeMode);
		int startDepthStream(ImiFrameMode depthFrameMode);

		void stopAllStreams();

		void stopColorStream();
		void stopUVCStream();
		void stopDepthStream();

		int openDevice();
		int closeDevice();

		void setColorFrameCallback(FrameCallbackFunction callback);
		void setUVCFrameCallback(FrameCallbackFunction callback);
		void setDepthFrameCallback(FrameCallbackFunction callback);
        void setCloudPointCallback(CloudCallbackFunction callback);

        void convertToCloudPoint(ImiImageFrame* pFrame);

        sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height);

		static int readFrame(void* lParam);
        static int readUVCFrame(void* lParam);

	private:
        boost::mutex device_mutex_;
        boost::mutex uvc_mutex_;
		ImiDeviceAttribute* pDeviceAttr;
		ImiDeviceHandle pImiDevice;

		ImiStreamHandle depthHandle;
		ImiStreamHandle colorHandle;
		ImiCameraHandle camHandle;

		bool isDeviceValid;
		int color_video_started_;
		int depth_video_started_;
		int uvc_video_started_;

		FrameCallbackFunction depthCallback;
		FrameCallbackFunction colorCallback;
        CloudCallbackFunction cloudCallback;
		FrameCallbackFunction uvcCallback;

        double m_cx;
        double m_cy;
        double m_fx;
        double m_fy;
	};
}

#endif /* OPENNI_DEVICE_H */
