
#include <iostream>

#include "ORBSLAM/include/Camera.h"

using namespace std;

namespace ORB_SLAM2 {
  std::string Camera::name;
  float Camera::width;
  float Camera::height;
  float Camera::fx;
  float Camera::fy;
  float Camera::cx;
  float Camera::cy;
  float Camera::invfx;
  float Camera::invfy;
  float Camera::bf;
  float Camera::b;
  cv::Mat Camera::K;
  cv::Mat Camera::DistCoef;
  bool Camera::initialized = false;

  
  bool Camera::Load(const std::string strSettingPath) {
	cerr << endl << "Loading camera calibration from: " << strSettingPath << endl;
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	Camera::Load(fSettings);
	fSettings.release();
	return true;
  }

  bool Camera::Load(cv::FileStorage fSettings) {
	Camera::width = fSettings["Camera.width"];
	Camera::height = fSettings["Camera.height"];
	Camera::fx = fSettings["Camera.fx"];
	Camera::fy = fSettings["Camera.fy"];
	Camera::cx = fSettings["Camera.cx"];
	Camera::cy = fSettings["Camera.cy"];
  
	cv::Mat K = cv::Mat::eye(3,3,CV_32F);
	K.at<float>(0,0) = fx;
	K.at<float>(1,1) = fy;
	K.at<float>(0,2) = cx;
	K.at<float>(1,2) = cy;
	K.copyTo(Camera::K);
  
	cv::Mat DistCoef(4,1,CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if(k3!=0)
	  {
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	  }
	DistCoef.copyTo(Camera::DistCoef);
  
	Camera::bf = fSettings["Camera.bf"];

	Camera::invfx = 1.0f/Camera::fx;
	Camera::invfy = 1.0f/Camera::fy;
	Camera::b = Camera::bf/Camera::fx;

	Camera::initialized = true;
	cout << "- size: " << Camera::width << "x" <<  Camera::height << endl;
	cout << "- fx: " << Camera::fx << endl;
	cout << "- fy: " << Camera::fy << endl;
	cout << "- cx: " << Camera::cx << endl;
	cout << "- cy: " << Camera::cy << endl;
	cout << "- k1: " << Camera::DistCoef.at<float>(0) << endl;
	cout << "- k2: " << Camera::DistCoef.at<float>(1) << endl;
	if(Camera::DistCoef.rows==5)
	  cout << "- k3: " << Camera::DistCoef.at<float>(4) << endl;
	cout << "- p1: " << Camera::DistCoef.at<float>(2) << endl;
	cout << "- p2: " << Camera::DistCoef.at<float>(3) << endl;
	cout << "- bf: " << Camera::bf << endl;
	return true;
  }

}



