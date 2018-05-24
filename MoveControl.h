#ifndef   MOVECONTROL_H
#define   MOVECONTROL_H
#include "UART_Fuc.h"
#include <math.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include "CostMap/include/utils.h"
#include "ORBSLAM/include/System.h"

#define  TREAD              0.335
#define  OUT_RADIUS         0.4
#define  CYCLETIME          0.02


class MoveControl
{
public:
    MoveControl();
    ~MoveControl();

    HRGPara::Pose2D   m_CurrPos_Credible;

    HRGPara::Pose2D   m_Curr_ORB_POS;
    HRGPara::Pose2D   m_Curr_Odo_POS;
    cv::Mat           m_Curr_ORB_Rcw;
    cv::Mat           m_Curr_ORB_Ow;

    HRGPara::Pose2D   m_Last_ORB_POS;
    HRGPara::Pose2D   m_Last_Odo_POS;
    cv::Mat           m_Last_ORB_Rcw;
    cv::Mat           m_Last_ORB_Ow;

    HRGPara::Pose2D   m_Curr_Odo_Ve;
    HRGPara::Pose2D   m_Last_Odo_Ve;

    HRGPara::Pose2D   m_Velocity;

    void  InitMoveControl(ORB_SLAM2::System  *pORBSystem);
    void  GetPosVelocity( HRGPara::Pose2D  &CurrPos,  HRGPara::Pose2D  &Velocity, cv::Mat  &Rcw, cv::Mat  &Ow);


    void  AnalysisVeocity(HRGPara::Pose2D  AssumptionVelocity, float &LeftSteer_Velocity,  float &RightSteer_Velocity);
    void  SendVeloCommToMotorControl(float &LeftSteer_Velocity,  float &RightSteer_Velocity);

private:
    //串口类
    UART_Fuc             m_cUART;
    boost::thread       *m_pthread;
    ORB_SLAM2::System   *m_pORBSystem;
    boost::mutex         m_data_mutex_;
    bool                 m_bRun;

    cv::Mat              m_Rcw;
    cv::Mat              m_Ow;

    float                m_fTread;

    void  Monitor_function();
    void  GetPosDatafromMat(cv::Mat Rcw, cv::Mat Ow, float &x_, float &y_, float &Angle_);
    bool  UpdataCurCarPos_ByOdometry(HRGPara::Pose2D tempOdoData, HRGPara::Pose2D  &CurCar_pos,  cv::Mat &Rcw, cv::Mat &Ow );
    bool  getusart(int *recv);
    bool  CalculateVelocity( HRGPara::Pose2D Pos_Start, HRGPara::Pose2D Pos_End,
                             double time_dif, HRGPara::Pose2D &Velocity_Loc);

    bool  GetOdometryData(HRGPara::Pose2D &OdometryData);
    void  SetCurCarPos(HRGPara::Pose2D  &CurCar_pos,  cv::Mat  &Rcw, cv::Mat  &Ow);
};


#endif // MOVECONTROL_H
