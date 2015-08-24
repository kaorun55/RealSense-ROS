/******************************************************************************
	INTEL CORPORATION PROPRIETARY INFORMATION
	This software is supplied under the terms of a license agreement or nondisclosure
	agreement with Intel Corporation and may not be copied or disclosed except in
	accordance with the terms of that agreement
	Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/

#pragma once
# ifndef RS_IMAGE_VIEW_NODELET
# define RS_IMAGE_VIEW_NODELET

///////////////////////////////////////////////
/// Dependencies
///////////////////////////////////////////////

#include <nodelet/nodelet.h>

namespace realsense
{
	///////////////////////////////////////////////
	///	CImageViewerNodelet - view images of realsense cameras
	///////////////////////////////////////////////
	class CImageViewerNodelet : public nodelet::Nodelet
	{
	public :
		//===================================
		//	Interface
		//===================================
		virtual void onInit();

		~CImageViewerNodelet();

	private:
		//===================================
		//	Member Functions
		//===================================

		//Static member functions:
		void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg);
		void imageColorCallback(const sensor_msgs::ImageConstPtr& msg);
		void uvmapCallback(const std_msgs::Float32MultiArrayConstPtr uv);

		void PublishDepthTextured(cv::Mat& imageColorRgb, cv::Mat& imageDepth, std::vector<float>& uv);

		//===================================
		//	Member Variables
		//===================================

		image_transport::Subscriber m_sub_depth;
		image_transport::Subscriber m_sub_color;
		ros::Subscriber m_image_uv_sub;

		cv::Mat mDepth;
		cv::Mat mColor;
		std::vector<float> m_uvmap;

	};
}


#endif // RS_IMAGE_VIEW_NODELET

