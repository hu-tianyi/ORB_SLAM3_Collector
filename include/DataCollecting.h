//
// Created by tianyi on 6/15/23.
//

#ifndef ORB_SLAM3_DATACOLLECTING_H
#define ORB_SLAM3_DATACOLLECTING_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include <iomanip>
#include <ctime>

#include <thread>
#include <cmath>
#include <numeric>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

#include "Frame.h"
#include "KeyFrame.h"
#include "sophus/se3.hpp"


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

    // Main function
    void Run();

    void RequestFinish();

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
    void CollectCurrentFrameMapPointDepth(const Frame &frame);
    //void CollectCurrentFrame;
    //void CollectCurrentKeyFrame(KeyFrame currentKeyFrame);

    void CollectLocalMappingBANumber(const int num_FixedKF_BA, const int num_OptKF_BA,
                                                     const int num_MPs_BA,  const int num_edges_BA);

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

    void CollectCurrentTime();
    void InitializeCSVLogger();
    void WriteRowCSVLogger();
    std::string msSeqName;
    std::string msCurrentTime;

    // Declare the mFileLogger as an reference using the & operator
    // So that I can copy the actual ofstream to it.
    // Settings of the .csv file
    std::ofstream mFileLogger;
    std::string msCSVFileName;
    std::vector<std::string> mvsColumnFeatureNames = {"Counter", "TimeStamp", "ImgFileName",\
                                                   "Brightness", "Contrast", "Entropy",\
                                                   "AvgMPDepth", "VarMPDepth", \
                                                   "TrackMode", "PrePOOutlier", "PrePOKeyMapLoss",\
                                                   "Inlier", "PostPOOutlier", "PostPOKeyMapLoss", "MatchedInlier",\
                                                   "NumberKeyPoints", "PX", "PY", "PZ", "Q1", "Q2", "Q3", "Q4", \
                                                   "DX", "DY", "DZ", "Yaw", "Pitch", "Roll",\
                                                   "num_FixedKF_BA", "num_OptKF_BA", "num_MPs_BA", "num_edges_BA"};

    // Mutexs for locks
    std::mutex mMutexNewFrameProcessed;

    std::mutex mMutexImageTimeStamp;
    std::mutex mMutexImageFileName;
    std::mutex mMutexImageCounter;
    std::mutex mMutexImagePixel;
    std::mutex mMutexImageFeatures;
    
    std::mutex mMutexCurrentFrame;
    std::mutex mMutexCurrentFrameTrackMode;
    std::mutex mMutexCurrentFramePrePOOutlier;
    std::mutex mMutexCurrentFramePrePOKeyMapLoss;
    std::mutex mMutexCurrentFrameInlier;
    std::mutex mMutexCurrentFramePostPOOutlier;
    std::mutex mMutexCurrentFramePostPOKeyMapLoss;
    std::mutex mMutexCurrentFrameMatchedInlier;
    std::mutex mMutexCurrentFrameMapPointDepth;
    std::mutex mMutexCurrentFrameFeatures;

    std::mutex mMutexLocalMappingBANumber;
    //std::mutex mMutex;


    // binary flags for data collection status
    bool mbImageFeaturesReady;
    bool mbCurrentFrameFeaturesReady;
    bool mbIsNewFrameProcessed;
    // TODO: bool mbLocalMapFeaturesReady;


    // member variables for data collection
    // input features
    double mdTimeStamp;
    string msImgFileName;
    int mnImCounter;
    cv::Mat mImGrey;
    double mdBrightness;
    double mdContrast;
    double mdEntropy;

    // tracking features
    int mnTrackMode;
    int mnPrePOOutlier;
    int mnPrePOKeyMapLoss;
    int mnInlier;
    int mnPostPOOutlier;
    int mnPostPOKeyMapLoss;
    int mnMatchedInlier;

    Frame mCurrentFrame;
    int mnkeypoints;
    vector<float> mvMapPointMinDepth;
    float mfMapPointAvgMinDepth;
    float mfMapPointVarMinDepth;
    // Current camera pose in world reference
    Sophus::SE3f mTwc;
    // World Pose
    Eigen::Quaternionf mQ; //= Twc.unit_quaternion();
    Eigen::Vector3f mtwc; //= Twc.translation();
    // Relative Pose
    Eigen::Quaternionf mRQ;
    Eigen::Vector3f mRtwc;
    Eigen::Vector3f mREuler;

    KeyFrame mCurrentKeyFrame;

    int mnFixedKF_BA;
    int mnOptKF_BA;
    int mnMPs_BA;
    int mnEdges_BA;


    // Private functions to process the collected data
    double CalculateImageEntropy(const cv::Mat& image);
    void CalculateImageFeatures();

    void CalculateCurrentFrameFeatures();

};


}


#endif //ORB_SLAM3_DATACOLLECTING_H
