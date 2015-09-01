/******************************************************************************
	INTEL CORPORATION PROPRIETARY INFORMATION
	This software is supplied under the terms of a license agreement or nondisclosure
	agreement with Intel Corporation and may not be copied or disclosed except in
	accordance with the terms of that agreement
	Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/Float32MultiArray.h"

#include "realsense_image_viewer_nodelet.h"

//Nodelet dependencies
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(realsense::CImageViewerNodelet, nodelet::Nodelet)

namespace realsense
{


	//******************************
	// Public Methods
	//******************************

	CImageViewerNodelet::~CImageViewerNodelet()
	{
		cv::destroyAllWindows();
	}

	void CImageViewerNodelet::onInit()
	{

		ros::NodeHandle& nh = getNodeHandle();
		cv::namedWindow("viewDepth");
		cv::namedWindow("viewColor");
		cv::namedWindow("viewDepthTextured");
		cv::startWindowThread();

		image_transport::ImageTransport it(nh);
		m_sub_depth = it.subscribe("camera/depth/image_raw", 1, &CImageViewerNodelet::imageDepthCallback, this);
		m_sub_color = it.subscribe("camera/color/image_raw", 1, &CImageViewerNodelet::imageColorCallback, this);
		m_image_uv_sub = nh.subscribe("camera/depth/uv", 1, &CImageViewerNodelet::uvmapCallback, this);

		ROS_INFO_STREAM("Starting RealSense Image Viewer node");

		return;
	 }

	//******************************
	// Private Methods
	//******************************

	void CImageViewerNodelet::imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		// Simply cliping the values so we can view them in gray scale.
		try
		{
			cv::Mat image =  cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
			cv::Mat imageDepth(image.rows,image.cols, CV_8UC1, cv::Scalar(0));

			image.convertTo(imageDepth, CV_8UC1, 255.0/2500.0 );

			mDepth = imageDepth;
			cv::imshow("viewDepth", imageDepth);

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
		}
	}

	void CImageViewerNodelet::imageColorCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			cv::Mat imageColor =  cv_bridge::toCvShare(msg, "bgr8")->image;

			mColor = imageColor;
			cv::imshow("viewColor", imageColor);
			PublishDepthTextured(mColor, mDepth, m_uvmap);

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}

	void CImageViewerNodelet::uvmapCallback(const std_msgs::Float32MultiArrayConstPtr uv)
	{
		//Just saving the data in a member field to use later
		m_uvmap = uv->data;
	}

	void CImageViewerNodelet::PublishDepthTextured(cv::Mat& imageColorRgb, cv::Mat& imageDepth, std::vector<float>& uv)
	{

		//This is an example how to color the depth image with the color data

		if (imageColorRgb.cols <= 0 || imageDepth.cols <= 0)
		{
			return;
		}

		uchar* colorData = imageColorRgb.data;

		cv::Mat imageDepthTextured(imageDepth.rows, imageDepth.cols, CV_8UC3, cv::Scalar(0,0,0));
		uchar* imageData = imageDepthTextured.data;

		float uvPixel[2];

		for (int y = 0; y < imageDepth.rows ; y++)
		{
			for ( int x = 0; x < imageDepth.cols; x++)
			{
				int index = x * 2 + y * imageDepth.cols * 2;

				uvPixel[0] = uv[index];
				uvPixel[1] = uv[index + 1];

				if (uvPixel[1] <= 0 || uvPixel[1] >= 1 ||
						uvPixel[0] <= 0 || uvPixel[0] >= 1 )
				{
					// no data
					imageDepthTextured.at<cv::Vec3b>(y,x)[0] = 0;
					imageDepthTextured.at<cv::Vec3b>(y,x)[1] = 0;
					imageDepthTextured.at<cv::Vec3b>(y,x)[2] = 0;
				}
				else
				{
					int i = (int)(uvPixel[0] * imageColorRgb.cols) ;
					int j = (int)(uvPixel[1] * imageColorRgb.rows) ;

					imageDepthTextured.at<cv::Vec3b>(y,x)[0] = imageColorRgb.at<cv::Vec3b>(j,i)[0];
					imageDepthTextured.at<cv::Vec3b>(y,x)[1] = imageColorRgb.at<cv::Vec3b>(j,i)[1];
					imageDepthTextured.at<cv::Vec3b>(y,x)[2] = imageColorRgb.at<cv::Vec3b>(j,i)[2];
				}

			}

		}

		cv::imshow("viewDepthTextured", imageDepthTextured);
	}
}
