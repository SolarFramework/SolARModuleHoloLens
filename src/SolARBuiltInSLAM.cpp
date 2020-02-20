#include "SolARBuiltInSLAM.h"

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
	// TODO
	//img(imageRPC.width(), imageRPC.height(), Image::ImageLayout::LAYOUT_RGB, Image::PixelOrder::INTERLEAVED, Image::DataType::TYPE_8U);
	return imgDest;
}

PoseMatrix ParseMatRPC(MatRPC matRPC)
{
	PoseMatrix mat;
	mat(0, 0) = matRPC.m11();
	mat(0, 1) = matRPC.m12();
	mat(0, 2) = matRPC.m13();
	mat(0, 3) = matRPC.m14();
	mat(1, 0) = matRPC.m21();
	mat(1, 1) = matRPC.m22();
	mat(1, 2) = matRPC.m23();
	mat(1, 3) = matRPC.m24();
	mat(2, 0) = matRPC.m31();
	mat(2, 1) = matRPC.m32();
	mat(2, 2) = matRPC.m33();
	mat(2, 3) = matRPC.m34();
	mat(3, 0) = matRPC.m41();
	mat(3, 1) = matRPC.m42();
	mat(3, 2) = matRPC.m43();
	mat(3, 3) = matRPC.m44();

	return mat;
}

PoseMatrix ParsePoseRPC(PoseRPC poseRPC)
{
	PoseMatrix camProj = ParseMatRPC(poseRPC.cameraproj());
	PoseMatrix camView = ParseMatRPC(poseRPC.cameraview());
	PoseMatrix frameToOrigin = ParseMatRPC(poseRPC.frametoorigin());

	PoseMatrix pose;
	// TODO check formula
	//if (!camProj.all()) // Projection matrix is null
	//{
	pose = camView * frameToOrigin;
	//}
	//else
	//{
	//	pose = camProj * camView * frameToOrigin;
	//}
	return pose;
}

SolARBuiltInSLAM::SolARBuiltInSLAM() : ConfigurableBase(xpcf::toUUID<SolARBuiltInSLAM>())
{
    addInterface<api::input::devices::IBuiltInSLAM>(this);
	declareProperty<std::string>("deviceAddress", m_deviceAddress);
	declareProperty<int>("isProxy", m_isProxy);
}

SolARBuiltInSLAM::~SolARBuiltInSLAM()
{
	stop();
}

xpcf::XPCFErrorCode SolARBuiltInSLAM::onConfigured()
{
	m_isClientConnected = false;
	return xpcf::XPCFErrorCode::_SUCCESS;
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

/// Obsolete
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
	//m_reader = m_stub->SensorStream(&m_context, MakeNameRPC(camera_name));
	
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

FrameworkReturnCode SolARBuiltInSLAM::getIntrinsics(const std::string & camera_name, CameraParameters & camParams)
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

}
}
}
