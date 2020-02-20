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

		SRef<Image> frame;
		PoseMatrix pose;
        std::pair<SRef<Image>, PoseMatrix> framePose;
		Transform3Df a;

		bool hasStartedCapture = false;

		// Buffers
		xpcf::DropBuffer<std::pair<SRef<Image>, PoseMatrix>>	m_dropBufferSensorCapture;
        xpcf::DropBuffer<SRef<Image>>							m_dropBufferDisplay;

		// Main Loop
		bool stop = false;
		clock_t start, end;
        int count = 0;

	// ADD HERE: The pipeline initialization
		LOG_INFO("Starting connection");
		// Connect remotely to the HoloLens streaming app
		if (slamHoloLens->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Can't connect to HoloLens");
			return -1;
		}
		LOG_INFO("Connection started!");
		// Retrieve camera intrinsics parameters
		if (slamHoloLens->getIntrinsics(camera_name, camParams) == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("request failed");
			return -1;
		}
		else
			LOG_INFO("got intrinsics \n{}", camParams.intrinsic);
		// Note: Camera distortion is assumed to be ideal coming from HoloLens sensors (undistortion is already performed internally)
		//overlay3D->setCameraParameters(camParams.intrinsic, camParams.distorsion);

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
			FrameworkReturnCode status = slamHoloLens->ReadCapture(frame, pose);
			LOG_DEBUG("Read capture done");
			switch (status)
			{
			case FrameworkReturnCode::_ERROR_:
				LOG_ERROR("Error during capture");
				stop = true;
				break;
			case FrameworkReturnCode::_SUCCESS:
				framePose = std::make_pair(frame, pose);
				LOG_DEBUG("\n{}", pose);
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
			std::pair<SRef<Image>, PoseMatrix> sensorFrame;
			if (!m_dropBufferSensorCapture.tryPop(sensorFrame))
			{
				xpcf::DelegateTask::yield();
				return;
			}
			SRef<Image> frame = sensorFrame.first;
			PoseMatrix pose = sensorFrame.second;
            // TO FIX
            // overlay3D->draw(pose, frame);
			m_dropBufferDisplay.push(frame);
		};

		// Display task
		auto fnDisplay = [&]()
		{
			SRef<Image> view;
            if (!m_dropBufferDisplay.tryPop(view))
			{
				xpcf::DelegateTask::yield();
				return;
			}
			//if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
			//{
			//	stop = true;
			//}
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
