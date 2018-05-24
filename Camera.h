/**
* This file could become part of ORB-SLAM2.
*
*/

#ifndef CAMERA_H
#define CAMERA_H

#include<opencv2/core/core.hpp>

namespace ORB_SLAM2 {

  class Camera {
  public:
	static bool Load(const std::string strSettingPath);
	static bool Load(cv::FileStorage fSettings);
	
	static std::string name;
	static float width;
	static float height;
	static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
	static float bf;          // Stereo baseline multiplied by fx.
	static float b;           // Stereo baseline in meters
	static cv::Mat K;
	static cv::Mat DistCoef;
	static bool initialized;
  };


} // namespace ORB_SLAM2

#endif // CAMERA_H
