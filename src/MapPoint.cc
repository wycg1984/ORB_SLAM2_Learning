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
* MapPoint是地图中的特征点 在orb slam中，就是通过MapPoint类来维护共视关系的
* 在这个类中它需要完成的主要工作有以下方面：
* 1）维护关键帧之间的共视关系
* 2）通过计算描述向量之间的距离，在多个关键帧的特征点中找最匹配的特征点
* 3）在闭环完成修正后，需要根据修正的主帧位姿修正特征点
* 4）对于非关键帧，也产生MapPoint，只不过是给Tracking功能临时使用
*/

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;
//构造函数 关键帧地图点构造函数 Pos指的是该点的3D位置，pRefKF是参考关键帧，pMap是地图
MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}
//构造函数 普通帧地图点构造函数 其中Pos指的是该点的3D位置，pMap是地图，pFrame是对应的普通帧，idxF是地图点在该帧特征点中的索引号。
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}
//增加地图点观测关系 pKF是对应的关键帧 idx是该地图点在关键帧中对应的索引值
void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    //如果已经存在观测关系，就返回
    //std::map<KeyFrame *, size_t>  mObservations 它是用来存放观测关系的容器，把能够观测到该MapPoint的关键帧，以及MapPoint在该关键帧中对应的索引值关联并存储起来
    if(mObservations.count(pKF))
        return;
    //如果不存在，就添加
    mObservations[pKF]=idx;
    //分成单目和双目两种清空添加观测，单目时观测次数加1，双目时观测次数加2
    //nObs 它用来记录被观测的次数
    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}
//删除地图点观测关系  pKF指的是关键帧
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        //判断该关键帧是否在观测关系中，即该关键帧是否看到了这个MapPoint
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];

            //这里同样要判断单目和双目，单目时观测次数减1，双目时减2
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;
            //删除该关键帧对应的观测关系
            mObservations.erase(pKF);
            //如果关键帧是参考帧则重新指定
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            // 当被观测次数小于等于2时，该地图点需要剔除
            if(nObs<=2)
                bBad=true;
        }
    }
     //即删除地图点
    if(bBad)
        SetBadFlag();
}
//
map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}
//删除地图点
void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        //清除该地图点所有的观测关系
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        //删除关键帧中和该MapPoint对应的匹配关系
        pKF->EraseMapPointMatch(mit->second);
    }
    //从地图中删除MapPoint
    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}
//替换地图点 pMP就是用来替换的地图点
void MapPoint::Replace(MapPoint* pMP)
{
    //如果传入的该MapPoint就是当前的MapPoint，直接跳出
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;
        // 如果该MapPoint不在关键帧的观测关系中，就添加观测关系
        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else //如果在就删除关键帧和老的MapPoint之间的对应关系
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();
    //删掉Map中该地图点
    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}
//计算描述子
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        //如果地图点标记为不好，直接返回
        if(mbBad)
            return;
        observations=mObservations;
    }
    //如果观测为空，则返回
    if(observations.empty())
        return;
    //保留的描述子数最多和观测数一致
    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            //针对每帧的对应的都提取其描述子
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    // 选择距离其他描述子中值距离最小的描述子作为地图点的描述子，基本上类似于取了个均值
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}
//更新法向量和深度值
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        //观测点坐标减去关键帧中相机光心的坐标就是观测方向
        //也就是说相机光心指向地图点
        cv::Mat normali = mWorldPos - Owi;
        //对其进行归一化后相加
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;
    //深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取金字塔放大尺度
    //得到最大距离mfMaxDistance;最大距离除以整个金字塔最高层的放大尺度得到最小距离mfMinDistance.
    //通常说来，距离较近的地图点，将在金字塔较高的地方提出，
    //距离较远的地图点，在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）
    //因此，通过地图点的信息（主要对应描述子），我们可以获得该地图点对应的金字塔层级
    //从而预测该地图点在什么范围内能够被观测到
    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}
//预测尺度 currentDist是当前距离，pKF是关键帧
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}
//预测尺度
int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
