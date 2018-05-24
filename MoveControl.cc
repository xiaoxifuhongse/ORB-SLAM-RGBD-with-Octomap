#include "dataStruct.h"
#include "ORBSLAM/include/MoveControl.h"

using namespace HRGPara;




///////////High CRC///////////
static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

///////////Low CRC///////////
static unsigned char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen )
{
    unsigned char uchCRCHi = 0xFF;
    unsigned char uchCRCLo = 0xFF;
    unsigned uIndex ;
    while (usDataLen--)
    {
        uIndex = uchCRCLo ^ *puchMsg++;
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}


MoveControl::MoveControl()
{
    m_pthread = NULL;
    m_pORBSystem = NULL;
    m_bRun    = true;
    m_fTread  = TREAD;
}

MoveControl::~MoveControl()
{
    m_bRun = false;
    m_pthread->interrupt();
    m_pthread->join();
    delete  m_pthread;
}


bool MoveControl::GetOdometryData(HRGPara::Pose2D &OdometryData)
{
    bool bReadOK = false;

    int  index=0;
    while( !bReadOK && (index<10) )
    {
        int drecv[3];
        memset(drecv,0,3*sizeof(int));
        usleep(5000);
        if( getusart(drecv) )
        {
            bReadOK = true;

            HRGPara::Pose2D temp;
            temp.point.x   = double(drecv[0])/10000;
            temp.point.y   = double(drecv[1])/10000;
            temp.alfa      = double(drecv[2])/100;
            int dNum = temp.alfa / 360;
            if( abs(dNum) >= 1 )
            {
                if(dNum > 0)
                    temp.alfa -= dNum * 360;

                if(dNum < 0)
                    temp.alfa -= (dNum - 1) * 360;
            }
            else
            {
                 if(temp.alfa < 0)
                     temp.alfa += 360;
            }

            HRGPara::Pose2D  tempPose(temp.point.x, temp.point.y, temp.alfa);
            OdometryData = tempPose;
        }
        index++;
    }

    return bReadOK;
}

void MoveControl::InitMoveControl(ORB_SLAM2::System *pORBSystem)
{
    m_pORBSystem = pORBSystem;
    m_pthread    = new boost::thread(boost::bind(&MoveControl::Monitor_function, this));

}

bool MoveControl::UpdataCurCarPos_ByOdometry(HRGPara::Pose2D tempOdoData, HRGPara::Pose2D  &CurCar_pos, cv::Mat &Rcw, cv::Mat &Ow )
{
    //update the Pose2D
    float x_   = tempOdoData.point.x;
    float y_   = tempOdoData.point.y;
    float Ang_ = tempOdoData.alfa;

    float OffSet_X = x_ - m_Curr_Odo_POS.point.x;
    float OffSet_y = y_ - m_Curr_Odo_POS.point.y;
    float e = (m_Curr_Odo_POS.alfa*PI/(double)CONVERSION_PARA);
    float Deleta_X   =  OffSet_X*cos(e) + OffSet_y*sin(e);
    float Deleta_Y   = -OffSet_X*sin(e) + OffSet_y*cos(e);
    float Deleta_Ang =  Ang_ - m_Curr_Odo_POS.alfa;

    CurCar_pos.point.x  = Deleta_X   + m_Curr_ORB_POS.point.x;
    CurCar_pos.point.y  = Deleta_Y   + m_Curr_ORB_POS.point.y;
    CurCar_pos.alfa     = Deleta_Ang + m_Curr_ORB_POS.alfa;

    int dNum = CurCar_pos.alfa / 360;
    if( abs(dNum) >= 1 )
    {
        if(dNum > 0)
            CurCar_pos.alfa -= dNum * 360;

        if(dNum < 0)
            CurCar_pos.alfa -= (dNum - 1) * 360;
    }
    else
    {
         if(CurCar_pos.alfa < 0)
             CurCar_pos.alfa += 360;
    }

    //update the Matrix (rotate&offset)
    if(!m_Curr_ORB_Ow.empty()&&!m_Curr_ORB_Rcw.empty())
    {
        Ow = m_Curr_ORB_Ow.clone();
        Ow.ptr<float>(0)[0] = CurCar_pos.point.x;
        Ow.ptr<float>(2)[0] = CurCar_pos.point.y;

        tf::Matrix3x3 tf3d_Reference;
        tf3d_Reference.setValue(
                      m_Curr_ORB_Rcw.ptr<float>(0)[0], m_Curr_ORB_Rcw.ptr<float>(0)[1] ,m_Curr_ORB_Rcw.ptr<float>(0)[2],
                      m_Curr_ORB_Rcw.ptr<float>(1)[0], m_Curr_ORB_Rcw.ptr<float>(1)[1] ,m_Curr_ORB_Rcw.ptr<float>(1)[2],
                      m_Curr_ORB_Rcw.ptr<float>(2)[0], m_Curr_ORB_Rcw.ptr<float>(2)[1] ,m_Curr_ORB_Rcw.ptr<float>(2)[2]
                      );

        double roll,pitch, yaw;
        tf3d_Reference.getEulerYPR(yaw, pitch, roll);

        pitch += Deleta_Ang * PI/ CONVERSION_PARA;
        tf::Matrix3x3  tf3d_New;
        tf3d_New.setRPY(roll,pitch, yaw);

        Rcw = m_Curr_ORB_Rcw.clone();
        Rcw.ptr<float>(0)[0] = tf3d_New.getRow(0).x();
        Rcw.ptr<float>(0)[1] = tf3d_New.getRow(0).y();
        Rcw.ptr<float>(0)[2] = tf3d_New.getRow(0).z();

        Rcw.ptr<float>(1)[0] = tf3d_New.getRow(1).x();
        Rcw.ptr<float>(1)[1] = tf3d_New.getRow(1).y();
        Rcw.ptr<float>(1)[2] = tf3d_New.getRow(1).z();

        Rcw.ptr<float>(2)[0] = tf3d_New.getRow(2).x();
        Rcw.ptr<float>(2)[1] = tf3d_New.getRow(2).y();
        Rcw.ptr<float>(2)[2] = tf3d_New.getRow(2).z();
    }

    return true;
}

void  MoveControl::SendVeloCommToMotorControl(float &LeftSteer_Velocity,  float &RightSteer_Velocity)
{
    short int  Left_wheel_Vel  = (short int)(LeftSteer_Velocity * 1000);
    short int  Right_wheel_Vel = (short int)(RightSteer_Velocity * 1000);

    unsigned char send_buf[13];
    memset(send_buf, 0, sizeof(send_buf));

    send_buf[0]   = 0xaa;
    send_buf[1]   = 0x55;
    send_buf[2]   = 0x01;
    send_buf[3]   = 0x00;
    send_buf[4]   = 0x04;
    send_buf[5]   = 0x02;
    send_buf[6]   = 0x11;

    //left wheel velocity
    send_buf[7]   = (unsigned char)(Left_wheel_Vel >> 8) & 0xFF;
    send_buf[8]   = (unsigned char)(Left_wheel_Vel) & 0xFF;

    //right wheel velocity
    send_buf[9]  = (unsigned char)(Right_wheel_Vel >> 8) & 0xFF;
    send_buf[10]   = (unsigned char)(Right_wheel_Vel) & 0xFF;

    short int CRC_Code = CRC16(send_buf, 11 );

    //CRC check
    send_buf[11]  = (unsigned char)(CRC_Code >> 8) & 0xFF;
    send_buf[12]  = (unsigned char)(CRC_Code) & 0xFF;

    m_cUART.UART_Send( send_buf, sizeof(send_buf));
}

bool  MoveControl::getusart(int *recv)
{
    bool  bDataValid = false;
    unsigned char message[UART_BUF_LEN];
    memset(message,0,UART_BUF_LEN);
    m_cUART.UART_Read(message);


    for(int i=0; i<UART_BUF_LEN;i++)
    {
        if(message[i] == 0xdd)
        {
            if(message[i+1] == 0x66)
            {
                int  flag1 = message[i+2];
                int  Pos_x = (int)((long int)(message[i+3])+ ((long int)((message[i+4]))<<8) + ((long int)((message[i+5]))<<16) );
                *recv      = ( flag1 ? (Pos_x * -1) : Pos_x );

                int  flag2 = message[i+6];
                int  Pos_y = (int)((long int)(message[i+7])+ ((long int)((message[i+8]))<<8) + ((long int)((message[i+9]))<<16) );
                *(recv+1)  = ( flag2 ? (Pos_y * -1) : Pos_y );

                int  flag3 = message[i+10];
                int  Pos_ang = (int)((long int)(message[i+11])+ ((long int)((message[i+12]))<<8) );
                *(recv+2)  = ( flag3 ? (Pos_ang * -1) : Pos_ang );

                //和校验
                int sum = 0;
                for(int j=i; j< i+13; j++)
                {
                    sum += (int)message[j];
                }
                unsigned char CheckResultA =  ( unsigned char )(sum) & 0xFF;
                unsigned char CheckResultB =  ( unsigned char )message[i+13];
                if(CheckResultA == CheckResultB)
                    bDataValid = true;
                break;

            }
        }
    }

    if( bDataValid == false )
        cout<<"Notice: Read Odometry data failed!"<<endl;

    return bDataValid;
}

void  MoveControl::AnalysisVeocity( HRGPara::Pose2D  AssumptionVelocity, float &LeftSteer_Velocity,  float &RightSteer_Velocity )
{
    double planPos_x = CYCLETIME * AssumptionVelocity.point.x;
    double planPos_y = CYCLETIME * AssumptionVelocity.point.y;

    //know:  Pos1(0,0)  Pos2(planPos_x, planPos_y);  reslove:(x-a)2+y2 = r2
    if( (planPos_y!=0) && (planPos_x!=0) )
    {
        double center_a = (planPos_x*planPos_x + planPos_y*planPos_y)/(2 * planPos_x);
        double radius   = abs(center_a);

        double half_chordlength = sqrt(planPos_x*planPos_x + planPos_y*planPos_y)/2;
        double Ang = 2 * asin(half_chordlength / radius);

        double arclength = Ang * radius;
        double V_mean    = arclength / CYCLETIME;

        float  V_leftwheel = 0;
        float  V_rightwheel= 0;
        if(center_a > 0)
        {
            V_leftwheel  = ( radius + TREAD/2 )*V_mean/radius;
            V_rightwheel = ( radius - TREAD/2 )*V_mean/radius;
        }
        else
        {
            V_rightwheel= ( radius + TREAD/2 )*V_mean/radius;
            V_leftwheel = ( radius - TREAD/2 )*V_mean/radius;
        }

        LeftSteer_Velocity  = V_leftwheel;
        RightSteer_Velocity = V_rightwheel;
    }
    else if( (planPos_y!=0) && (planPos_x==0) )
    {
        LeftSteer_Velocity  = planPos_y;
        RightSteer_Velocity = planPos_y;
    }
    else
    {
        LeftSteer_Velocity  = 0;
        RightSteer_Velocity = 0;
    }
}


bool  MoveControl::CalculateVelocity( HRGPara::Pose2D Pos_Start, HRGPara::Pose2D Pos_End,
                                      double time_dif, HRGPara::Pose2D &Velocity_Loc )
{
    bool  flag = false;

    double V_X        = Pos_End.point.x -  Pos_Start.point.x;
    double V_Y        = Pos_End.point.y -  Pos_Start.point.y;
    double Deleta_Ang = Pos_End.alfa - Pos_Start.alfa;

    double e        = (Pos_Start.alfa*PI/(double)CONVERSION_PARA);
    double Deleta_X =  V_X*cos(e) + V_Y*sin(e);
    double Deleta_Y = -V_X*sin(e) + V_Y*cos(e);

    double Velocity_Ang = 0;
    double Velocity_X   = 0;
    double Velocity_Y   = 0;

    if(time_dif != 0.0)
    {
        Velocity_Ang = Deleta_Ang/time_dif;
        Velocity_X   = Deleta_X/time_dif;
        Velocity_Y   = Deleta_Y/time_dif;

        boost::unique_lock<boost::mutex> lock( m_data_mutex_ );
        Velocity_Loc.point.x = Velocity_X;
        Velocity_Loc.point.y = Velocity_Y;
        Velocity_Loc.alfa    = Velocity_Ang;
        lock.unlock();

        flag =true;
    }

    return flag;
}

void  MoveControl::GetPosVelocity( HRGPara::Pose2D  &CurrPos,  HRGPara::Pose2D  &Velocity, cv::Mat  &Rcw, cv::Mat  &Ow )
{
    boost::unique_lock<boost::mutex> lock( m_data_mutex_ );
    CurrPos = m_CurrPos_Credible;
    Velocity = m_Velocity;
    Rcw = m_Rcw.clone();
    Ow  = m_Ow.clone();
    lock.unlock();
}


void  MoveControl::SetCurCarPos(HRGPara::Pose2D  &CurCar_pos,  cv::Mat  &Rcw, cv::Mat  &Ow)
{
    boost::unique_lock<boost::mutex> lock( m_data_mutex_ );
    m_CurrPos_Credible = CurCar_pos;
    m_Rcw = Rcw.clone();
    m_Ow  = Ow.clone();
    lock.unlock();
}

void  MoveControl::Monitor_function()
{
    double time_Start = 0;
    double time_End   = 0;
    double time_dif   = 0;

    while(m_bRun)
    {
        double sleepTime_Start = HRGPara::GetSystemTime();

        HRGPara::Pose2D tempOdometryData;
        bool flag1 = GetOdometryData(tempOdometryData);

        time_End = HRGPara::GetSystemTime();
        time_dif = time_Start - time_End;

        cv::Mat Rcw, Ow;

        //bool flag2;
        bool flag2 = m_pORBSystem->GetRobotPos(Rcw, Ow);

        if(flag1 && flag2)
        {
            float x_, y_, Angle_;
            GetPosDatafromMat(Rcw, Ow, x_, y_, Angle_);

            cout<<"MoveControl(ORB valid): "<<x_<<" "<<y_<<" "<<Angle_<<endl;
            HRGPara::Pose2D   Curr_ORB_POS_Credible(x_, y_, Angle_);

            //refresh the ORB & Odo information
            m_Last_ORB_POS = m_Curr_ORB_POS;
            m_Curr_ORB_POS = Curr_ORB_POS_Credible;
            SetCurCarPos(m_Curr_ORB_POS, Rcw, Ow);

            m_Last_Odo_POS = m_Curr_Odo_POS;
            m_Curr_Odo_POS = tempOdometryData;

            m_Last_ORB_Rcw = m_Curr_ORB_Rcw.clone();
            m_Curr_ORB_Rcw = Rcw.clone();
            m_Last_ORB_Ow  = m_Curr_ORB_Ow.clone();
            m_Curr_ORB_Ow  = Ow.clone();

            m_Curr_Odo_Ve  = m_Curr_Odo_POS;
            m_Last_Odo_Ve  = m_Last_Odo_POS;

            //refresh the velocity data
            CalculateVelocity( m_Last_Odo_Ve, m_Curr_Odo_Ve, time_dif, m_Velocity );

        }
        else if(flag1 && !flag2)
        {
            HRGPara::Pose2D  CurCar_pos;
            cv::Mat Rcw_,Ow_;
            UpdataCurCarPos_ByOdometry(tempOdometryData, CurCar_pos,  Rcw_, Ow_);

            //refresh the ORB & Odo information
            SetCurCarPos(CurCar_pos, Rcw_, Ow_);
            cout<<"Ow:"<<Ow<<endl;

            m_Last_Odo_Ve  = m_Curr_Odo_Ve;
            m_Curr_Odo_Ve  = tempOdometryData;

            //refresh the velocity data
            CalculateVelocity( m_Last_Odo_Ve, m_Curr_Odo_Ve, time_dif, m_Velocity );

            cout<<"MoveControl(ORB invalid): "<<CurCar_pos.point.x<<" "<<CurCar_pos.point.y<<" "<<CurCar_pos.alfa<<endl;
        }


        time_Start = time_End;

        double sleepTime_End = HRGPara::GetSystemTime();
        double sleepTime_Dif = sleepTime_End - sleepTime_Start;

        double plan_cycletime = CYCLETIME * 1000000;
        double real_sleeptime = plan_cycletime - sleepTime_Dif;

        usleep(real_sleeptime);
    }
}

void  MoveControl::GetPosDatafromMat(cv::Mat Rcw, cv::Mat Ow, float &x_, float &y_, float &Angle_)
{
    cv::Mat  Rwc = Rcw.t();

    HRGPara::Point  CP_car_head(0,0,1);
    HRGPara::Point  WP_car_head;

    WP_car_head.x  = Rwc.ptr<float>(0)[0] * CP_car_head.x + Rwc.ptr<float>(0)[1] * CP_car_head.y + Rwc.ptr<float>(0)[2] * CP_car_head.z;
    WP_car_head.y  = Rwc.ptr<float>(1)[0] * CP_car_head.x + Rwc.ptr<float>(1)[1] * CP_car_head.y + Rwc.ptr<float>(1)[2] * CP_car_head.z;
    WP_car_head.z  = Rwc.ptr<float>(2)[0] * CP_car_head.x + Rwc.ptr<float>(2)[1] * CP_car_head.y + Rwc.ptr<float>(2)[2] * CP_car_head.z;

    float  fAxisX_x = WP_car_head.x;
    float  fAxisX_y = WP_car_head.z;

    unsigned int  dphaseX;   // 1, 2, 3, 4,  5 =X+, 6=Y+, 7=X-, 8=Y-
    dphaseX =0;
    if( (fAxisX_x > 0) && (fAxisX_y > 0) )
    {
        dphaseX = 1;
    }
    else if( (fAxisX_x < 0 ) && (fAxisX_y > 0) )
    {
        dphaseX = 2;
    }
    else if( (fAxisX_x < 0 ) && (fAxisX_y < 0) )
    {
        dphaseX = 3;
    }
    else if( (fAxisX_x > 0 ) && (fAxisX_y < 0) )
    {
        dphaseX = 4;
    }
    else if( (fAxisX_x > 0 ) && (fAxisX_y == 0) )
    {
        dphaseX = 5;
    }
    else if( (fAxisX_x == 0 ) && (fAxisX_y > 0) )
    {
        dphaseX = 6;
    }
    else if( (fAxisX_x < 0 ) && (fAxisX_y == 0) )
    {
        dphaseX = 7;
    }
    else if( (fAxisX_x == 0 ) && (fAxisX_y < 0) )
    {
        dphaseX = 8;
    }
    else if( (fAxisX_x == 0 ) && (fAxisX_y == 0) )
    {
        dphaseX = 5;
    }

    float fAngle_X;
    // 5 =X+, 6=Y+, 7=X-, 8=Y-
    switch (dphaseX)
    {
    case 1:
        fAngle_X = atan( fAxisX_y/fAxisX_x ) * 180 / 3.1415926;
        break;

    case 2:
        fAngle_X = atan( fAxisX_y/fAxisX_x ) * 180 / 3.1415926 + 180;
        break;

    case 3:
        fAngle_X = atan( fAxisX_y/fAxisX_x ) * 180 / 3.1415926 + 180;
        break;

    case 4:
        fAngle_X = atan( fAxisX_y/fAxisX_x ) * 180 / 3.1415926 + 360;
        break;

    case 5:
        fAngle_X = 0;
        break;

    case 6:
        fAngle_X = 90;
        break;

    case 7:
        fAngle_X = 180;
        break;

    case 8:
        fAngle_X = 270;
        break;

    default:
        break;
    }

    x_     = Ow.ptr<float>(0)[0];
    y_     = Ow.ptr<float>(2)[0];
    Angle_ = fAngle_X;

    //cout<<"MoveControl:"<<Angle_<<endl;

    /*
     x_ = 1 * Ow.ptr<float>(0)[0];
     y_ = 1 * Ow.ptr<float>(2)[0];

     float r3_1 = Rcw.ptr<float>(2)[0];
     float r3_2 = Rcw.ptr<float>(2)[1];
     float r3_3 = Rcw.ptr<float>(2)[2];
     float r1_1 = Rcw.ptr<float>(0)[0];
     float r2_1 = Rcw.ptr<float>(1)[0];

     float Angle_B = asin( -r3_1 ) * 180 / 3.1415926;
     //float Angle_R = atan( r3_2 / r3_3 ) * 180 / 3.1415926;
     float Angle_A = atan( r2_1 / r1_1 ) * 180 / 3.1415926;


     //bool  bsinB = -r3_1 > 0;
     bool  bcosB = (r2_1 > 0)^(Angle_A > 0) ? 0 : 1;

     float  val_cos = bcosB ? -sqrtf( r2_1*r2_1 + r1_1*r1_1 ) : sqrtf( r2_1*r2_1 + r1_1*r1_1 );
     float  val_sin = -r3_1;


     if( (val_sin > 0) && (val_cos > 0) );
     else if( (val_sin > 0) && (val_cos < 0) )
         Angle_B = 180 - Angle_B;
     else if( (val_sin < 0) && (val_cos > 0) )
         Angle_B = 360 + Angle_B;
     else if( (val_sin < 0) && (val_cos < 0) )
         Angle_B = 180 - Angle_B;
     else if( (val_sin == 0) && (val_cos == 1) )
         Angle_B = 0;
     else if( (val_sin == 1) && (val_cos == 0) )
         Angle_B = 90;
     else if( (val_sin == 0) && (val_cos == -1) )
         Angle_B = 180;
     else if( (val_sin == -1) && (val_cos == 0) )
         Angle_B = 270;

     Angle_ = 270 - Angle_B;

     if(Angle_ < 0)
         Angle_ +=360;//change

     if(Angle_ > 360)
         Angle_ -=360;//change

     */

}

