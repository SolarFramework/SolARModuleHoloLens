#include "SolARHoloLensHelper.h"
#include "SolAROpenCVHelper.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <core/Log.h>

using SolAR::MODULES::OPENCV::SolAROpenCVHelper;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace HOLOLENS {

FrameworkReturnCode SolARHoloLensHelper::rotateImage(SRef<Image>& src, SRef<Image>& dst, int angle)
{
	/// From https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
	cv::Mat img = SolAROpenCVHelper::mapToOpenCV(src);
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
	return SolAROpenCVHelper::convertToSolar(out, dst);
}

Transform3Df SolARHoloLensHelper::fromPoseMatrix(PoseMatrix mat)
{
	Transform3Df t;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			t(i, j) = mat(i, j);
	return t.inverse();
}

std::string SolARHoloLensHelper::ParseNameRPC(NameRPC name)
{
	return name.cameraname();
}

NameRPC SolARHoloLensHelper::MakeNameRPC(std::string cameraName)
{
	NameRPC name;
	name.set_cameraname(cameraName);
	return name;
}

CamCalibration SolARHoloLensHelper::ParseCameraIntrinsicsRPC(CameraIntrinsicsRPC camIntrinsics)
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

SRef<Image> SolARHoloLensHelper::ParseImageRPC(ImageRPC imageRPC)
{
	SRef<Image> imgSrc;
	uint8_t* dataPointer = (uint8_t*)imageRPC.data().c_str();
	imgSrc = xpcf::utils::make_shared<Image>(dataPointer, imageRPC.width(), imageRPC.height(), Image::ImageLayout::LAYOUT_GREY, Image::PixelOrder::INTERLEAVED, Image::DataType::TYPE_8U);
	// Convert from grayscale to RGB using OpenCV
	cv::Mat imgGray(imgSrc->getHeight(), imgSrc->getWidth(), CV_8UC1, imgSrc->data());
	cv::Mat imgColor;
	cv::cvtColor(imgGray, imgColor, cv::COLOR_GRAY2RGB);
	SRef<Image> imgDest;
	imgDest = xpcf::utils::make_shared<Image>(imgColor.ptr(), imgColor.cols, imgColor.rows, Image::ImageLayout::LAYOUT_RGB, Image::PixelOrder::INTERLEAVED, Image::DataType::TYPE_8U);
	return imgDest;
}

PoseMatrix SolARHoloLensHelper::ParseMatRPC(MatRPC matRPC)
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

Transform3Df SolARHoloLensHelper::ParsePoseRPC(PoseRPC poseRPC)
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
		pose = cameraToImage * camView * frameToOrigin_inv;
	else
		LOG_WARNING("Matrix not invertible, invalid pose");
	return fromPoseMatrix(pose);
}

bool SolARHoloLensHelper::IsValidPose(Transform3Df pose)
{
	return !pose.isApprox(Transform3Df());
}

}
}
}