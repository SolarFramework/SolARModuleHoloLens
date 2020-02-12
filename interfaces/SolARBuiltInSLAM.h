#ifndef SOLARBUILTINSLAM_H
#define SOLARBUILTINSLAM_H

#include "SolARModuleHoloLensAPI.h"
#include "api/input/devices/IBuiltInSLAM.h"
#include "datastructure/CameraDefinitions.h"
#include "datastructure/Image.h"
#include "xpcf/component/ConfigurableBase.h"

#include "sensorStreaming.grpc.pb.h"
#include "sensorStreaming.pb.h"

#include <string>
#include <vector>

namespace xpcf = org::bcom::xpcf;

using sensorStreaming::Streamer;

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
    ~SolARBuiltInSLAM() override;

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
    FrameworkReturnCode getLastCapture(std::vector<Image> & frames, std::vector<PoseMatrix> & poses) override;

    /// @brief Retrieve the sensors intrinsic parameters. (assuming all sensors are cameras, ie no IMU)
    /// @return Vector containing all available sensors intrinsics.
    CameraParameters & getIntrinsics(const std::string & camera_name) override;

    /// @brief Whether or not the device is simulated or online.
    /// @return True is the device is online, False is simulated.
	bool isProxy() override { return m_isProxy; };

    /// @brief Set IP and port for remote connection to the device.
    void setConnectionParams(const std::string & ip, const std::string & port) override;

private:
	std::string m_deviceAddress;
	bool m_isProxy;
	std::unique_ptr<Streamer::Stub> m_stub;

	std::vector<std::string> m_sensorList;
};

}
}
}

#endif // SOLARBUILTINSLAM_H
