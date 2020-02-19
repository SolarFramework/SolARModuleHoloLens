#ifndef SOLARBUILTINSLAM_H
#define SOLARBUILTINSLAM_H

#include "SolARModuleHoloLensAPI.h"
#include "api/input/devices/IBuiltInSLAM.h"
#include "datastructure/CameraDefinitions.h"
#include "datastructure/Image.h"
#include "xpcf/component/ConfigurableBase.h"

#include "grpc/grpc.h"
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

#include "sensorStreaming/sensorStreaming.grpc.pb.h"
#include "sensorStreaming/sensorStreaming.pb.h"

#include <string>
#include <vector>

namespace xpcf = org::bcom::xpcf;

// From gRPC proto file
using sensorStreaming::Streamer;
using sensorStreaming::NameRPC;
using sensorStreaming::CameraIntrinsicsRPC;
using sensorStreaming::PoseRPC;
using sensorStreaming::MatRPC;
using sensorStreaming::ImageRPC;
using sensorStreaming::SensorFrameRPC;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace HOLOLENS {

/**
  * @class SolARBuiltInSLAM
  * @brief <B>Retrieve synchronized sensor images and poses from an external device (ie. HoloLens).</B>
  * <TT>UUID: b5f5f897-1f0e-4268-be2d-c344170690e8 </TT>
  *
  * Component for retrieving data from a device that has a built-in SLAM capability.
  */
class SOLAR_HOLOLENS_EXPORT_API SolARBuiltInSLAM : public xpcf::ConfigurableBase,
	public api::input::devices::IBuiltInSLAM
{
public:
    /// @brief Specify the IBuiltInSLAM constructor class
    SolARBuiltInSLAM();

    /// @brief Specify the IBuiltInSLAM destructor class
    ~SolARBuiltInSLAM();

    xpcf::XPCFErrorCode onConfigured() override final;

    void unloadComponent() override final;

    /// @brief Start the connection to the device for sensors data streaming.
    /// @return FrameworkReturnCode::_SUCCESS if successful, eiher FrameworkReturnCode::_ERROR_.
    FrameworkReturnCode start() override;

    /// @brief Stop the connection to the device.
    /// @return FrameworkReturnCode::_SUCCESS if successful, eiher FrameworkReturnCode::_ERROR_.
    FrameworkReturnCode stop() override;

    /// @brief Fill frames and poses vectors containing latest sensors data from the device.
    /// @return FrameworkReturnCode to track successful or failing event.
    FrameworkReturnCode getLastCapture(std::vector<SRef<Image>> & frames, std::vector<PoseMatrix> & poses) override;

    /// @brief Retrieve the sensors intrinsic parameters. (assuming all sensors are cameras, ie no IMU)
    /// @return FrameworkReturnCode to track successful or failing event.
	/// Obsolete
	FrameworkReturnCode getIntrinsics(const std::string & camera_name, CameraParameters & camParams) override;

	FrameworkReturnCode RequestCapture(const std::string & camera_name) override;

	FrameworkReturnCode ReadCapture(SRef<Image> & frame, PoseMatrix & pose) override;

    /// @brief Whether or not the device is simulated or online.
    /// @return True is the device is online, False is simulated.
	bool isProxy() override { return m_isProxy == 1; };

private:
	std::string m_deviceAddress;
	int m_isProxy;
	std::unique_ptr<Streamer::Stub> m_stub;
	grpc::ClientContext m_context;
	std::unique_ptr<grpc::ClientReader<SensorFrameRPC>> m_reader;
	bool m_isClientConnected;

	std::vector<std::string> m_sensorList;
};

}
}
}

#endif // SOLARBUILTINSLAM_H
