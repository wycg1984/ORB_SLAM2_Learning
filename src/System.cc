/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{
/**系统的构造函数，包括所有功能模块和所有线程的初始化 
 * strVocFile: ORB词袋数据 词袋是在做闭环检测和重定位时候做检索用的，从历史关键帧中检索出和当前帧最相似的一帧，以进行位姿匹配。
 * strSettingsFile: 配置文件的路径。配置文件中存放了相机参数和用户显示界面的设置
 * sensor: 传感器类型，这是一个枚举变量，一共有三种，分别是MONOCULAR、STEREO和RGBD，分别代表单目、双目和RGBD。
 * bUseViewer: true代表启动显示界面，false代表不显示
*/
System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary 加载ORB词袋模型 
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    //初始化词袋对象
    mpVocabulary = new ORBVocabulary();
    //从词袋数据文件中读取数据
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database 创建关键帧数据集，数据集中主要存放词袋模型里的值，这些值用于闭环检测和重定位
    //使用词袋数据创建数据集对象
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map 创建地图
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer画关键帧的drawer
    mpFrameDrawer = new FrameDrawer(mpMap);
    //画地图的drawer strSettingsFile: 配置文件的路径。配置文件中存放了相机参数和用户显示界面的设置
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread 创建跟踪线程
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch 创建局部地图线程
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    //打开局部地图对应的线程
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch 创建闭环检测线程
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    //打开对应的线程
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch 根据标志位判断是否需要显示
    if(bUseViewer)
    {
        //显示界面
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        //打开显示对应的线程
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads 给各个线程之间建立关联
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}
//双目(相机)数据入口
cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        //检测是否激活纯定位模式
        if(mbActivateLocalizationMode)
        {
            //停止局部地图构建
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            //设置成纯定位模式
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        //检测是否退出纯定位模式
        if(mbDeactivateLocalizationMode)
        {
            //取消纯定位模式
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    //检测是否需要重启
    if(mbReset)
    {
        //重启
        mpTracker->Reset();
        mbReset = false;
    }
    }
    //执行双目跟踪 三个参数：左目、右目和时间戳
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}
//RGBD数据入口
cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        //检测是否激活纯定位模式
        if(mbActivateLocalizationMode)
        {
            //停止局部地图构建
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            //设置成纯定位模式
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
         //检测是否退出纯定位模式
        if(mbDeactivateLocalizationMode)
        {
            //取消纯定位模式
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    //检测是否需要重启
    if(mbReset)
    {
        //重启
        mpTracker->Reset();
        mbReset = false;
    }
    }
    //执行RGBD跟踪 函数参数：像素图、深度图和时间戳
    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}
//单目数据入口
cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        //检测是否激活纯定位模式
        if(mbActivateLocalizationMode)
        {
            //停止局部地图构建
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            //设置成纯定位模式
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        //检测是否退出纯定位模式
        if(mbDeactivateLocalizationMode)
        {
            //取消纯定位模式
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    //检测是否需要重启
    if(mbReset)
    {
        //重启
        mpTracker->Reset();
        mbReset = false;
    }
    }
    //执行单目跟踪 函数参数:像素图和时间戳
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
