#include "SolARBuiltInSLAM.h"

#include <core/Log.h>

#include "grpc/grpc.h"
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

// From gRPC proto file
using sensorStreaming::Streamer;
using sensorStreaming::NameRPC;
using sensorStreaming::CameraIntrinsicsRPC;
using sensorStreaming::PoseRPC;
using sensorStreaming::ImageRPC;
using sensorStreaming::SensorFrameRPC;

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

NameRPC MakeNameRPC(std::string cameraName)
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

Image ParseImageRPC(ImageRPC imageRPC)
{
	Image img(imageRPC.width(), imageRPC.height(), Image::ImageLayout::LAYOUT_RGB, Image::PixelOrder::INTERLEAVED, Image::DataType::TYPE_8U);
	return img;
}

PoseMatrix ParsePoseRPC(PoseRPC poseRPC)
{
	PoseMatrix pose;
	pose(0, 0) = poseRPC.m11();
	pose(0, 1) = poseRPC.m12();
	pose(0, 2) = poseRPC.m13();
	pose(0, 3) = poseRPC.m14();
	pose(1, 0) = poseRPC.m21();
	pose(1, 1) = poseRPC.m22();
	pose(1, 2) = poseRPC.m23();
	pose(1, 3) = poseRPC.m24();
	pose(2, 0) = poseRPC.m31();
	pose(2, 1) = poseRPC.m32();
	pose(2, 2) = poseRPC.m33();
	pose(2, 3) = poseRPC.m34();
	pose(3, 0) = poseRPC.m41();
	pose(3, 1) = poseRPC.m42();
	pose(3, 2) = poseRPC.m43();
	pose(3, 3) = poseRPC.m44();

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

	LOG_DEBUG("SolARBuiltInSLAM -> Starting gRPC client...")
	if (start() == FrameworkReturnCode::_SUCCESS)
	{
		LOG_DEBUG("SolARBuiltInSLAM -> gRPC client successfully initialized.")
		return xpcf::XPCFErrorCode::_SUCCESS;
	}
	LOG_WARNING("SolARBuiltInSLAM -> gRPC client failed to initialize.")
	return xpcf::XPCFErrorCode::_ERROR_NULL_POINTER;
}

FrameworkReturnCode SolARBuiltInSLAM::start()
{
	// Initializing stub with connection channel to the device
	std::unique_ptr<Streamer::Stub> m_stub(Streamer::NewStub(grpc::CreateChannel(m_deviceAddress, grpc::InsecureChannelCredentials())));
    // TODO upgrade to encrypted authentication to device //
	if (m_stub == nullptr)
	{
		LOG_WARNING("SolARBuiltInSLAM -> Can't initiate channel connection to device");
		return FrameworkReturnCode::_ERROR_;
	}
	m_isClientConnected = true;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::stop()
{
	// TODO Cleanup
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBuiltInSLAM::getLastCapture(std::vector<Image> & frames, std::vector<PoseMatrix> & pose)
{
	// One call should be enough to get a constant stream of data (?)
	SensorFrameRPC sensorFrame;
	grpc::ClientContext context;
	std::unique_ptr<grpc::ClientReader<SensorFrameRPC>> reader(
		m_stub->SensorStream(&context, MakeNameRPC("vlc_lf"))
	);
	// Read stream buffer
	while (reader->Read(&sensorFrame))
	{
		if (sensorFrame.has_image())
		{
			Image img = ParseImageRPC(sensorFrame.image());
		}
		if (sensorFrame.has_pose())
		{
			PoseMatrix pose = ParsePoseRPC(sensorFrame.pose());
		}
	}
	// Stream ended
	grpc::Status status = reader->Finish();
	if (!status.ok())
	{
		return FrameworkReturnCode::_ERROR_;
	}
	return FrameworkReturnCode::_SUCCESS;
}

CameraParameters & SolARBuiltInSLAM::getIntrinsics(const std::string & camera_name)
{
	grpc::ClientContext context;
	CameraIntrinsicsRPC camParamsRPC;
	CameraParameters camParams;
	grpc::Status status = m_stub->GetCamIntrinsics(&context, MakeNameRPC(camera_name), &camParamsRPC);
	if (status.ok())
	{
		camParams.intrinsic = ParseCameraIntrinsicsRPC(camParamsRPC);
	}
	return camParams;
}

}
}
}
