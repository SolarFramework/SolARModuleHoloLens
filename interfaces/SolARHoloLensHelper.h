#ifndef SOLARHOLOLENSHELPER_H
#define SOLARHOLOLENSHELPER_H

#include "SolARModuleHoloLensAPI.h"
#include "datastructure/Image.h"
#include "datastructure/CameraDefinitions.h"
#include "datastructure/MathDefinitions.h"
#include "xpcf/api/IComponentManager.h"
#include "core/Messages.h"

#include "opencv2/core.hpp"

#include "sensorStreaming/sensorStreaming.grpc.pb.h"
#include "sensorStreaming/sensorStreaming.pb.h"

namespace xpcf = org::bcom::xpcf;

// From gRPC proto file
using sensorStreaming::Streamer;
using sensorStreaming::NameRPC;
using sensorStreaming::SensorListRPC;
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
 * @class SolARHoloLensHelper
 * @brief A toolbox based on OpenCV to convert HoloLens/gRPC structures to SolAR
 */

class SOLAR_HOLOLENS_EXPORT_API SolARHoloLensHelper {
public:
	/// @brief Rotate the <src> by a given <angle> amount and put the result in <dst>
	/// @params[in] src The Image to be rotated
	/// @params[in,out] dst The destination Image that will store the rotated Image
	/// @params[in] angle The desired angle of rotations in degrees
	static FrameworkReturnCode rotateImage(SRef<Image>& src, SRef<Image>& dst, int angle);

	/// @brief Convert HoloLens camera pose to SolAR pose
	/// @params[in] mat The source HoloLens camera pose as a PoseMatrix
	/// @return The converted SolAR pose as a Transform3Df
	static Transform3Df fromPoseMatrix(PoseMatrix mat);

	/// @brief Convert sensor name from RPC to string
	static std::string ParseNameRPC(NameRPC name);

	/// @brief Create the RPC object with a given string sensor name
	static NameRPC MakeNameRPC(std::string camera_name);

	/// @brief Convert RPC CamIntrinsics to SolAR CamCalibration structure
	static CamCalibration ParseCameraIntrinsicsRPC(CameraIntrinsicsRPC CameraIntrinsicsRPC);

	/// @brief Read the raw data sent through RPC as a SolAR Image
	static SRef<Image> ParseImageRPC(ImageRPC imageRPC);

	/// @brief Convert a RPC matrix to a SolAR camera PoseMatrix
	static PoseMatrix ParseMatRPC(MatRPC matRPC);

	/// @brief Compute the SolAR camera pose from the RPC pose
	static Transform3Df ParsePoseRPC(PoseRPC poseRPC);

	/// @brief check whether the given pose matrix looks correct or not
	static bool IsValidPose(Transform3Df pose);
};

}
}
}

#endif // SOLARHOLOLENSHELPER_H
