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
	// Instantiate component manager and load the pipeline configuration file
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
		if(xpcfComponentManager->load("conf_Sample-HoloLens.xml")!=org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file conf_Sample-HoloLens.xml")
			return -1;
		}

	// ADD HERE: instantiate concrete components and bind them to abstract component interfaces
		// e.g. SRef<image::ICamera> camera = xpcfComponentManager->resolve<image::ICamera>();
		SRef<input::devices::IBuiltInSLAM> slamHoloLens = xpcfComponentManager->resolve<input::devices::IBuiltInSLAM>();
		SRef<display::IImageViewer> imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
        SRef<display::I3DOverlay> overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();

	// ADD HERE: Declare here the data structures used to connect components
		std::string camera_name = "vlc_lf";
		CameraParameters camParams;

		SRef<Image> frame;
		PoseMatrix pose;
		Transform3Df a;

		// Buffers
		xpcf::DropBuffer<std::pair<SRef<Image>, PoseMatrix>>	m_dropBufferSensorCapture;
        xpcf::DropBuffer<SRef<Image>>							m_dropBufferDisplay;

		// Main Loop
		bool stop = false;
		clock_t start, end;
        int count = 0;

	// ADD HERE: The pipeline initialization

		// Connect remotely to the HoloLens streaming app
		slamHoloLens->start();
		// Retrieve camera intrinsics parameters
		slamHoloLens->getIntrinsics(camera_name, camParams);
		// Note: Camera distortion is assumed to be ideal coming from HoloLens sensors (undistortion is already performed internally)
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distorsion);

		// Capture task
		auto fnCapture = [&]()
		{
			// TODO
			//Init sensor capture
			if (slamHoloLens->RequestCapture(camera_name) == FrameworkReturnCode::_ERROR_)
				return;

			// Read and update loop
			FrameworkReturnCode status = slamHoloLens->ReadCapture(frame, pose);
			if (status == FrameworkReturnCode::_ERROR_)
			{
				return;
			}
			else if (status == FrameworkReturnCode::_SUCCESS)
			{
				std::pair<SRef<Image>, PoseMatrix> framePose;
				framePose = std::make_pair(frame, pose);
				m_dropBufferSensorCapture.push(framePose);
			}
			else if (status == FrameworkReturnCode::_STOP)
			{
				return;
			}
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
			if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
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
		taskProcess.start();

		end = clock();
		// Display stats on frame rate
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);
	}
	catch (xpcf::Exception &e)
	{
		LOG_DEBUG("{}", e.what());
		return -1;
	}

    return 0;
}
