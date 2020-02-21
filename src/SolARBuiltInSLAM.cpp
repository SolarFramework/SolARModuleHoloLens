#include "SolARBuiltInSLAM.h"

#include <opencv2/core.hpp>

#include <core/Log.h>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::HOLOLENS::SolARBuiltInSLAM)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace HOLOLENS {

std::string ParseNameRPC(NameRPC name)
{
	return name.cameraname();
}

const NameRPC MakeNameRPC(std::string cameraName)
{
	NameRPC name;
	name.set_cameraname(cameraName);
	return name;
}

CamCalibration ParseCameraIntrinsicsRPC(CameraIntrinsicsRPC camIntrinsics)
{
	float fx = camIntrinsics.fx();
	float fy = camIntrinsics.fy();
	float cx = camIntrinsics.cx();
	float cy = camIntrinsics.cy();

	CamCalibration intrinsics;
	intrinsics(0, 0) = fx;
	intrinsics(1, 1) = fy;
	intrinsics(0, 2) = cx;
	intrinsics(1, 2) = cy;
	intrinsics(2, 2) = 1;

	return intrinsics;
}

SRef<Image> ParseImageRPC(ImageRPC imageRPC)
{
	SRef<Image> imgDest;
	uint8_t* dataPointer = (uint8_t*) imageRPC.data().c_str();
	imgDest = xpcf::utils::make_shared<Image>(dataPointer, imageRPC.width(), imageRPC.height(), Image::ImageLayout::LAYOUT_GREY, Image::PixelOrder::INTERLEAVED, Image::DataType::TYPE_8U);
	return imgDest;
}

PoseMatrix ParseMatRPC(MatRPC matRPC)
{
	PoseMatrix mat;
	mat(0, 0) = matRPC.m11();
	mat(0, 1) = matRPC.m21();
	mat(0, 2) = matRPC.m31();
	mat(0, 3) = matRPC.m41();
	mat(1, 0) = matRPC.m12();
	mat(1, 1) = matRPC.m22();
	mat(1, 2) = matRPC.m32();
	mat(1, 3) = matRPC.m42();
	mat(2, 0) = matRPC.m13();
	mat(2, 1) = matRPC.m23();
	mat(2, 2) = matRPC.m33();
	mat(2, 3) = matRPC.m43();
	mat(3, 0) = matRPC.m14();
	mat(3, 1) = matRPC.m24();
	mat(3, 2) = matRPC.m34();
	mat(3, 3) = matRPC.m44();

	return mat;
}

PoseMatrix ParsePoseRPC(PoseRPC poseRPC)
{
	PoseMatrix camProj = ParseMatRPC(poseRPC.cameraproj());
	PoseMatrix camView = ParseMatRPC(poseRPC.cameraview());
	PoseMatrix frameToOrigin = ParseMatRPC(poseRPC.frametoorigin());

	PoseMatrix pose = PoseMatrix::Zero();
	// cameraToImage flips Y and Z axis to comply with the output coordinate system.
	PoseMatrix cameraToImage = PoseMatrix::Identity();
	cameraToImage(1, 1) = -1;
	cameraToImage(2, 2) = -1;

	bool invert_ok;
	PoseMatrix frameToOrigin_inv;
	frameToOrigin.computeInverseWithCheck(frameToOrigin_inv, invert_ok);
	if (invert_ok)
	{
		pose = cameraToImage * camView * frameToOrigin_inv;
	}
	else
	{
		LOG_WARNING("Matrix not invertible, invalid pose")
	}
	return pose;
}

SolARBuiltInSLAM::SolARBuiltInSLAM() : ConfigurableBase(xpcf::toUUID<SolARBuiltInSLAM>())
{
    addInterface<api::input::devices::IBuiltInSLAM>(this);
	declareProperty<std::string>("deviceAddress", m_deviceAddress);
	declareProperty<std::string>("calibrationFile", m_calibrationFile);
	declareProperty<int>("isProxy", m_isProxy);
}

SolARBuiltInSLAM::~SolARBuiltInSLAM()
{
	stop();
}

xpcf::XPCFErrorCode SolARBuiltInSLAM::onConfigured()
{
	m_isClientConnected = false;
	m_sensorList.emplace_back("vlc_lf");

	if (m_calibrationFile.empty())
	{
		LOG_ERROR("Camera Calibration file path is empty");
		return xpcf::_FAIL;
	}
	cv::FileStorage fs(m_calibrationFile, cv::FileStorage::READ);
	cv::Mat intrinsic_parameters;
	cv::Mat distortion_parameters;
	int width, height;
	CameraParameters camParams;

	if (fs.isOpened())
	{
		for (auto sensor : m_sensorList)
		{
			auto params = fs[sensor];
			params["image_width"] >> width;
			params["image_height"] >> height;
			params["camera_matrix"] >> intrinsic_parameters;
			params["distortion_coefficients"] >> distortion_parameters;

			camParams.resolution.width = width;
			camParams.resolution.height = height;

			if (intrinsic_parameters.empty())
			{
				LOG_ERROR("No intrinsics found in calibration file");
					return xpcf::_FAIL;
			}

			if (intrinsic_parameters.rows == camParams.intrinsic.rows() && intrinsic_parameters.cols == camParams.intrinsic.cols())
				for (int i = 0; i < intrinsic_parameters.rows; i++)
					for (int j = 0; j < intrinsic_parameters.cols; j++)
						camParams.intrinsic(i, j) = (float)intrinsic_parameters.at<double>(i, j);
			else
			{
				LOG_ERROR("Camera Calibration should be a 3x3 Matrix")
					return xpcf::_FAIL;
			}

			if (distortion_parameters.empty())
			{
				LOG_ERROR("No distortion parameters found in calibration file")
					return xpcf::_FAIL;
			}

			if (distortion_parameters.rows == camParams.distorsion.rows() && distortion_parameters.cols == camParams.distorsion.cols())
				for (int i = 0; i < distortion_parameters.rows; i++)
					for (int j = 0; j < distortion_parameters.cols; j++)
						camParams.distorsion(i, j) = distortion_parameters.at<double>(i, j);
			else
			{
				LOG_ERROR("Camera distortion matrix should be a 5x1 Matrix")
					return xpcf::_FAIL;
			}

			m_camParameters.emplace_back(camParams);
		}
		return xpcf::_SUCCESS;
	}
	else
	{
		LOG_ERROR("Cannot open camera calibration file")
			return xpcf::_FAIL;
	}

	return xpcf::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::start()
{
	if (m_isClientConnected)
	{
		LOG_ERROR("Can't connect to device, client already connected!")
		return FrameworkReturnCode::_ERROR_;
	}
	// Initializing stub with connection channel to the device
	LOG_DEBUG("Starting gRPC client...");
	m_channel = grpc::CreateChannel(m_deviceAddress, grpc::InsecureChannelCredentials());
	m_stub = Streamer::NewStub(m_channel);
    // TODO upgrade to encrypted authentication to device //
	if (m_stub == nullptr)
	{
		LOG_ERROR("Can't initiate channel connection to device at {}", m_deviceAddress);
		return FrameworkReturnCode::_ERROR_;
	}
	LOG_DEBUG("gRPC client successfully connected to device");
	m_isClientConnected = true;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::stop()
{
	if (m_stub != nullptr)
	{
		m_stub = nullptr;
	}
	return FrameworkReturnCode::_SUCCESS;
}

/// Deprecated
FrameworkReturnCode SolARBuiltInSLAM::getLastCapture(std::vector<SRef<Image>> & frames, std::vector<PoseMatrix> & poses)
{
	if (!m_isClientConnected)
	{
		LOG_ERROR("Client not connected, can't satisfy request");
		return FrameworkReturnCode::_ERROR_;
	}

	SensorFrameRPC sensorFrame;
	grpc::ClientContext m_context;
	std::unique_ptr<grpc::ClientReader<SensorFrameRPC>> reader(
		m_stub->SensorStream(&m_context, MakeNameRPC("vlc_lf"))
	);
	// Read stream buffer
	while (reader->Read(&sensorFrame))
	{
		if (sensorFrame.has_image())
		{
			SRef<Image> img = ParseImageRPC(sensorFrame.image());
            frames.emplace_back(img);
		}
		if (sensorFrame.has_pose())
		{
			PoseMatrix pose = ParsePoseRPC(sensorFrame.pose());
            poses.emplace_back(pose);
		}
	}
	// Stream ended
	grpc::Status status = reader->Finish();
	if (!status.ok())
	{
		LOG_ERROR("Request ended abrubtly: {}", status.error_message());
		return FrameworkReturnCode::_ERROR_;
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::RequestCapture(const std::string & camera_name)
{
	if (!m_isClientConnected)
	{
		LOG_ERROR("Client not connected, can't satisfy request");
		return FrameworkReturnCode::_ERROR_;
	}
	LOG_DEBUG("Retrieving stream reader from the device...");
	
	m_capture = new ClientCall;
	m_capture->reader = m_stub->SensorStream(&m_capture->context, MakeNameRPC(camera_name));

	LOG_DEBUG("{}", m_capture->reader);
	if (m_capture->reader == nullptr)
	{
		LOG_ERROR("Reader is null");
		return FrameworkReturnCode::_ERROR_;
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::ReadCapture(SRef<Image> & frame, PoseMatrix & pose)
{
	if (!m_isClientConnected)
	{
		LOG_ERROR("Client not connected, can't satisfy request");
		return FrameworkReturnCode::_ERROR_;
	}

	if (m_capture->reader == nullptr)
	{
		LOG_ERROR("No reader available, did you request a capture?");
		return FrameworkReturnCode::_ERROR_;
	}

	SensorFrameRPC sensorFrame;
	bool res = m_capture->reader->Read(&sensorFrame);
	if (!res)
	{
		grpc::Status status = m_capture->reader->Finish();
		if (!status.ok())
		{
			LOG_ERROR("Request ended abrubtly: {}", status.error_message());
			return FrameworkReturnCode::_ERROR_;
		}
		LOG_DEBUG("Reader is empty, end of the stream");
		return FrameworkReturnCode::_STOP;
	}
	// Reader as new data that we can process
	if (sensorFrame.has_image())
	{
		frame = ParseImageRPC(sensorFrame.image());
	}
	if (sensorFrame.has_pose())
	{
		pose = ParsePoseRPC(sensorFrame.pose());
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::RequestIntrinsicsFromDevice(const std::string & camera_name, CameraParameters & camParams)
{
	if (!m_isClientConnected)
	{
		LOG_ERROR("Client not connected, can't satisfy request");
		return FrameworkReturnCode::_ERROR_;
	}
	LOG_DEBUG("getIntrinsics request");
	CameraIntrinsicsRPC camParamsRPC;
	grpc::Status status = m_stub->GetCamIntrinsics(&m_context, MakeNameRPC(camera_name), &camParamsRPC);
	LOG_DEBUG("getIntrinsics request end");
	if (status.ok())
	{
		camParams.intrinsic = ParseCameraIntrinsicsRPC(camParamsRPC);
		return FrameworkReturnCode::_SUCCESS;
	}
	LOG_ERROR("getIntrinsics request failed: {}", status.error_message());
	return FrameworkReturnCode::_ERROR_;
}

FrameworkReturnCode SolARBuiltInSLAM::getIntrinsics(const std::string & camera_name, CameraParameters & camParams)
{
	for (int i = 0; i < m_sensorList.size(); i++)
	{
		if (m_sensorList[i] == camera_name)
		{
			camParams = m_camParameters[i];
			return FrameworkReturnCode::_SUCCESS;
		}
	}
	LOG_ERROR("Can't find intrinsics for sensor {}", camera_name);
	return FrameworkReturnCode::_ERROR_;
}

}
}
}
