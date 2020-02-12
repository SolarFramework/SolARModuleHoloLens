#include "SolARBuiltInSLAM.h"

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

SolARBuiltInSLAM::SolARBuiltInSLAM() : ConfigurableBase(xpcf::toUUID<SolARBuiltInSLAM>())
{
    addInterface<api::input::devices::IBuiltInSLAM>(this);
	declareProperty<std::string>("deviceAddress", m_deviceAddress);
}

SolARBuiltInSLAM::~SolARBuiltInSLAM()
{
	stop();
}

xpcf::XPCFErrorCode SolARBuiltInSLAM::onConfigured()
{
    return xpcf::XPCFErrorCode();
}

FrameworkReturnCode SolARBuiltInSLAM::start()
{
	// Initializing stub with connection channel to the device
    // TODO upgrade to encrypted authentication to device
	Streamer::Stub m_stub(grpc::CreateChannel(m_deviceAddress, grpc::InsecureChannelCredentials()));
	return FrameworkReturnCode::_ERROR_;
}

FrameworkReturnCode SolARBuiltInSLAM::stop()
{
	return FrameworkReturnCode::_ERROR_;
}

FrameworkReturnCode SolARBuiltInSLAM::getLastCapture(std::vector<Image> & frames, std::vector<PoseMatrix> & pose)
{
	return FrameworkReturnCode::_ERROR_;
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

void  SolARBuiltInSLAM::setConnectionParams(const std::string & ip, const std::string & port)
{
	m_deviceAddress = ip + ":" + port;
}

}
}
}
