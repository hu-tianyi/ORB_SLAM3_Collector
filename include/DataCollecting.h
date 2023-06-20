//
// Created by tianyi on 6/15/23.
//

#ifndef ORB_SLAM3_DATACOLLECTING_H
#define ORB_SLAM3_DATACOLLECTING_H

#include<unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

#include "Frame.h"
#include "KeyFrame.h"


namespace  ORB_SLAM3
{
class System;
class Tracking;
class LocalMapping;
class LoopClosing;

class DataCollecting
{
public:
    DataCollecting(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void InitializeCSVLogger();

    // Main function
    void Run();

    void Release();

    // Public functions to collect data
    void CollectImageTimeStamp(const double &timestamp);
    void CollectImageFileName(string &filename);
    void CollectImagePixel(cv::Mat &imGray);

    void CollectCurrentFrame(const Frame &frame);
    void CollectCurrentFrameTrackMode(const int &bTrackMode);
    void CollectCurrentFramePrePOOutlier(const int &nPrePOOutlier);
    void CollectCurrentFramePrePOKeyMapLoss(const int &nPrePOKeyMapLoss);
    void CollectCurrentFrameInlier(const int &nInlier);
    void CollectCurrentFramePostPOOutlier(const int &nPostPOOutlier);
    void CollectCurrentFramePostPOKeyMapLoss(const int &nPostPOKeyMapLoss);
    void CollectCurrentFrameMatchedInlier(const int &nMatchedInlier);
    //void CollectCurrentFrame;
    //void CollectCurrentKeyFrame(KeyFrame currentKeyFrame);
    


protected:

    System *mpSystem;
    Atlas* mpAtlas;
    bool mbMonocular;
    bool mbInertial;

    bool mbFinished;

    // member pointers to the three main modules
    Tracking* mpTracker;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;

    // Mutexs for locks
    std::mutex mMutexImageTimeStamp;
    std::mutex mMutexImageFileName;
    std::mutex mMutexImageCounter;
    std::mutex mMutexImagePixel;
    
    std::mutex mMutexCurrentFrame;
    std::mutex mMutexCurrentFrameTrackMode;
    std::mutex mMutexCurrentFramePrePOOutlier;
    std::mutex mMutexCurrentFramePrePOKeyMapLoss;
    std::mutex mMutexCurrentFrameInlier;
    std::mutex mMutexCurrentFramePostPOOutlier;
    std::mutex mMutexCurrentFramePostPOKeyMapLoss;
    std::mutex mMutexCurrentFrameMatchedInlier;
    std::mutex mMutexCurrentFrameFeatures;
    //std::mutex mMutex;
    
    
    std::mutex mMutexImageStatistics;


    // binary flags for data collection status
    bool mbImageFeaturesReady;
    bool mbCurrentFrameFeaturesReady;
    // TODO: bool mbLocalMapFeaturesReady;


    // member variables for data collection
    // input features
    double mTimeStamp;
    string msImgFileName;
    int mnImCounter;
    cv::Mat mImGrey;
    double mdBrightness;
    double mdContrast;
    double mdEntropy;

    // tracking features
    int mnkeypoints;
    float mfMapPointAvgMinDepth;
    float mfMapPointVarMinDepth;
    int mnTrackMode;
    int mnPrePOOutlier;
    int mnPrePOKeyMapLoss;
    int mnInlier;
    int mnPostPOOutlier;
    int mnPostPOKeyMapLoss;
    int mnMatchedInlier;

    Frame mCurrentFrame;
    KeyFrame mCurrentKeyFrame;

    // Private functions to process the collected data
    double CalculateImageEntropy(const cv::Mat& image);
    void CalculateImageFeatures();

    float CalculateMapPointDepth(Frame frame);

    void CalculateCurrentFrameFeatures();

};


}


#endif //ORB_SLAM3_DATACOLLECTING_H
