/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Common headers
#include "xpcf/xpcf.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <boost/log/core.hpp>
#include <boost/thread/thread.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

// ADD HERE: Module traits headers. #include "SolARModuleOpencv_traits.h"
#include "SolARModuleHoloLens_traits.h"
#include "SolARModuleOpencv_traits.h"

// ADD HERE: Component interfaces header. e.g. #include "api/input/devices/ICamera.h"
#include "api/input/devices/IBuiltInSLAM.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"

// Namespaces
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

static std::map<std::tuple<uint32_t, std::size_t, uint32_t>, int> solar2cvTypeConvertMap = { {std::make_tuple(8,1,3),CV_8UC3},{std::make_tuple(8,1,1),CV_8UC1} };

static std::map<int, std::pair<Image::ImageLayout, Image::DataType>> cv2solarTypeConvertMap = { {CV_8UC3,{Image::ImageLayout::LAYOUT_BGR,Image::DataType::TYPE_8U}},
																									  {CV_8UC1,{Image::ImageLayout::LAYOUT_GREY,Image::DataType::TYPE_8U}} };

Transform3Df fromPoseMatrix(PoseMatrix mat)
{
	Transform3Df t;
	for (int i; i < 4; i++)
	{
		for (int j; j < 4; j++)
		{
			t(i, j) = mat(i, j);
		}
	}
	return t;
}

FrameworkReturnCode convertToSolar(cv::Mat&  imgSrc, SRef<Image>& imgDest)
{
	if (cv2solarTypeConvertMap.find(imgSrc.type()) == cv2solarTypeConvertMap.end() || imgSrc.empty()) {
		return FrameworkReturnCode::_ERROR_LOAD_IMAGE;
	}
	std::pair<Image::ImageLayout, Image::DataType> type = cv2solarTypeConvertMap.at(imgSrc.type());
	imgDest = xpcf::utils::make_shared<Image>(imgSrc.ptr(), imgSrc.cols, imgSrc.rows, type.first, Image::PixelOrder::INTERLEAVED, type.second);

	return FrameworkReturnCode::_SUCCESS;
}

int deduceOpenCVType(SRef<Image> img)
{
	// TODO : handle safe mode if missing map entry
	// is it ok when destLayout != img->ImageLayout ?
	return solar2cvTypeConvertMap.at(std::forward_as_tuple(img->getNbBitsPerComponent(), 1, img->getNbChannels()));
}

cv::Mat mapToOpenCV(SRef<Image> imgSrc)
{
	cv::Mat imgCV(imgSrc->getHeight(), imgSrc->getWidth(), deduceOpenCVType(imgSrc), imgSrc->data());
	return imgCV;
}

void rotateImg(SRef<Image>& src, SRef<Image>& dst, int angle)
{
	// From https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
	cv::Mat img = mapToOpenCV(src);
	// get rotation matrix for rotating the image around its center in pixel coordinates
	cv::Point2f center((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);
	cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
	// determine bounding rectangle, center not relevant
	cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), img.size(), angle).boundingRect2f();
	// adjust transformation matrix
	rot.at<double>(0, 2) += bbox.width / 2.0 - img.cols / 2.0;
	rot.at<double>(1, 2) += bbox.height / 2.0 - img.rows / 2.0;

	cv::Mat out;
	cv::warpAffine(img, out, rot, bbox.size());
	// Convert image back to SolAR
	convertToSolar(out, dst);
}

// Main function
int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif
    LOG_ADD_LOG_TO_CONSOLE();

	try
	{
        LOG_INFO("Load config...");
	// Instantiate component manager and load the pipeline configuration file
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
		if(xpcfComponentManager->load("conf_Sample-HoloLens.xml")!=org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file conf_Sample-HoloLens.xml");
			return -1;
		}
        LOG_INFO("Config loaded!");
        LOG_INFO("Start creating components");

	// ADD HERE: instantiate concrete components and bind them to abstract component interfaces
		// e.g. SRef<image::ICamera> camera = xpcfComponentManager->resolve<image::ICamera>();
		//SRef<input::devices::IBuiltInSLAM> slamHoloLens;
		auto slamHoloLens = xpcfComponentManager->resolve<input::devices::IBuiltInSLAM>();
        auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();

        LOG_INFO("Components created!");
	// ADD HERE: Declare here the data structures used to connect components
		std::string camera_name = "vlc_lf";
		CameraParameters camParams;

		bool hasStartedCapture = false;

		// Buffers
		xpcf::DropBuffer<std::pair<SRef<Image>, PoseMatrix>>	m_dropBufferSensorCapture;
        xpcf::DropBuffer<SRef<Image>>							m_dropBufferDisplay;

		// Main Loop
		bool stop = false;
		clock_t start, end;
        int count = 0;

		////////////// TEMP ////////////////

		float half_X = 0.05f;
		float half_Y = 0.05f;
		float Z = 0.1f;

		Transform3Df transform;
		transform.setIdentity();

		std::vector<Vector4f> parallelepiped;

		parallelepiped.push_back(transform * Vector4f(-half_X, -half_Y, 0.0f, 1.0f));
		parallelepiped.push_back(transform * Vector4f( half_X, -half_Y, 0.0f, 1.0f));
		parallelepiped.push_back(transform * Vector4f( half_X,  half_Y, 0.0f, 1.0f));
		parallelepiped.push_back(transform * Vector4f(-half_X,  half_Y, 0.0f, 1.0f));
		parallelepiped.push_back(transform * Vector4f(-half_X, -half_Y, -Z, 1.0f));
		parallelepiped.push_back(transform * Vector4f( half_X, -half_Y, -Z, 1.0f));
		parallelepiped.push_back(transform * Vector4f( half_X,  half_Y, -Z, 1.0f));
		parallelepiped.push_back(transform * Vector4f(-half_X,  half_Y, -Z, 1.0f));

		////////////// TEMP ////////////////


	// ADD HERE: The pipeline initialization
		LOG_INFO("Starting connection");
		// Connect remotely to the HoloLens streaming app
		if (slamHoloLens->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Can't connect to HoloLens");
			return -1;
		}
		LOG_INFO("Connection started!");

		// Enable device sensors, and prepare them for streaming (WIP)
		std::vector<std::string> sensorList;
		sensorList.emplace_back("vlc_lf");
		sensorList.emplace_back("vlc_rf");
		if (slamHoloLens->EnableSensors(sensorList) == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Error when enabling sensors");
			return -1;
		}
		LOG_INFO("Enabled sensors");

		// Load camera intrinsics parameters
		if (slamHoloLens->getIntrinsics(camera_name, camParams) == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Failed to load {} intrinsics", camera_name);
			return -1;
		}
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distorsion);
		LOG_INFO("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distorsion);

		// Capture task
		auto fnCapture = [&]()
		{
			//Init sensor capture
			if (!hasStartedCapture)
			{
				LOG_INFO("Requesting stream capture for camera {}", camera_name);
				if (slamHoloLens->RequestCapture(camera_name) == FrameworkReturnCode::_ERROR_)
				{
					LOG_ERROR("Error requesting capture");
					return -1;
				}
				hasStartedCapture = true;
				LOG_DEBUG("Start streaming...");
			}
			// Read and update loop
			SRef<Image> frameCap;
			PoseMatrix poseMatCap;
			std::pair<SRef<Image>, PoseMatrix> framePose;
			FrameworkReturnCode status = slamHoloLens->ReadCapture(frameCap, poseMatCap);
			switch (status)
			{
			case FrameworkReturnCode::_ERROR_:
				LOG_ERROR("Error during capture");
				stop = true;
				break;
			case FrameworkReturnCode::_SUCCESS:
				framePose = std::make_pair(frameCap, poseMatCap);
                LOG_DEBUG("\n{}", poseMatCap);
				m_dropBufferSensorCapture.push(framePose);
				break;
			case FrameworkReturnCode::_STOP:
				LOG_ERROR("End of capture");
				stop = true;
				break;
			}
			count++;
		};

		// Process (draw pose on image)
		auto fnProcess = [&]()
		{
			SRef<Image> frameProcess;
			PoseMatrix poseMatProcess;
			std::pair<SRef<Image>, PoseMatrix> sensorFrame;
			if (!m_dropBufferSensorCapture.tryPop(sensorFrame))
			{
				xpcf::DelegateTask::yield();
				return;
			}
			frameProcess = sensorFrame.first;
			poseMatProcess = sensorFrame.second;

			Transform3Df pose = fromPoseMatrix(poseMatProcess);

			// Manual overlay computation
			PoseMatrix K;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					K(i, j) = camParams.intrinsic(i, j);
				}
			}
			K(0, 3) = 0; K(1, 3) = 0; K(2, 3) = 0;
			K(3, 0) = 0; K(3, 1) = 0; K(3, 2) = 0; K(3, 3) = 1;
			PoseMatrix P = K * poseMatProcess;
			LOG_DEBUG("K\n{}", K);
			LOG_DEBUG("P\n{}", P);
			cv::Mat displayedImage;
			displayedImage = mapToOpenCV(frameProcess);
			Vector4f proj;
			cv::Point2f uv;
			for (auto point : parallelepiped)
			{
				proj = P * point;
				if (proj(2) != 0)
				{
					uv.x = proj(0) / proj(2);
					uv.y = proj(1) / proj(2);
					LOG_DEBUG("\nx: {}, y: {}", uv.x, uv.y);
					if (0 <= uv.x && uv.x < displayedImage.cols && 0 <= uv.y && uv.y < displayedImage.rows)
					{
						circle(displayedImage, uv, 8, cv::Scalar(128, 0, 128), -1);
					}
				}
			}
            overlay3D->draw(pose, frameProcess);

			// Rotate 90 degrees
			SRef<Image> rotatedFrame;
			rotateImg(frameProcess, rotatedFrame, -90);
			// Push to display buffer
			m_dropBufferDisplay.push(rotatedFrame);
		};

		// Display task
		auto fnDisplay = [&]()
		{
			SRef<Image> frameDisplay;
            if (!m_dropBufferDisplay.tryPop(frameDisplay))
			{
				xpcf::DelegateTask::yield();
				return;
			}
			if (imageViewer->display(frameDisplay) == SolAR::FrameworkReturnCode::_STOP)
			{
				stop = true;
			}
		};

		// Instantiate and start tasks
		xpcf::DelegateTask taskDisplay(fnDisplay);
		xpcf::DelegateTask taskProcess(fnProcess);

		taskDisplay.start();
		taskProcess.start();

	// ADD HERE: The pipeline processing
		start = clock();
		while (!stop)
		{
			// GetCapture
			fnCapture();
		}
		// End tasks
		taskDisplay.stop();
		taskProcess.stop();

		end = clock();
		// Display stats on frame rate
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);
	}
	catch (xpcf::Exception &e)
	{
        LOG_ERROR("{}", e.what());
		return -1;
	}

    return 0;
}
