//
// Created by tianyi on 6/15/23.
//

#include "DataCollecting.h"

#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

namespace ORB_SLAM3
{

DataCollecting::DataCollecting(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
        mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial)
{

}


void DataCollecting::SetTracker(Tracking *pTracker)
{
    mpTracker = pTracker;
}

void DataCollecting::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void DataCollecting::SetLoopCloser(LoopClosing *pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}


void DataCollecting::InitializeCSVLogger()
{
    // TODO
}

void DataCollecting::Run()
{
    // Initialize the data collector
    mbFinished = false;
    mbImageFeaturesReady = false;
    mbCurrentFrameFeaturesReady = false;
    mnImCounter = 0;

    usleep(0.03*1000*1000);

    while(1)
    {
        //TODO
        if(mbImageFeaturesReady)
        {
            CalculateImageFeatures();
        }
        if(mbCurrentFrameFeaturesReady)
        {
            //CalculateCurrentFrameFeatures();
        }


        cout << "Testing Data Collection." << endl;
        {
            unique_lock<mutex> lock(mMutexImageTimeStamp);
            cout << "Current Time Stamp: " << setprecision(10) << mTimeStamp << endl; // prints 10 decimals
        }
        {
            unique_lock<mutex> lock(mMutexImageCounter);
            cout << "Current Image Counter: " << mnImCounter << endl; //
        }

        {
            unique_lock<mutex> lock(mMutexImageStatistics);
            //if(mbImageFeaturesReady)
            if(false)
            {
                cout << "Current Image Statistics: " << endl;
                cout << "Brightness:" << setprecision(5) << mdBrightness << endl; // prints 10 decimals
                cout << "Contrast:" << setprecision(5) << mdContrast << endl; // prints 10 decimals
                cout << "Entropy:" << setprecision(5) << mdEntropy << endl; // prints 10 decimals
            }
        }

        {
            unique_lock<mutex> lock(mMutexCurrentFrameFeatures);
            cout << "Current Frame Features: " << endl;
            cout << "Tracking Mode:" << mnTrackMode << endl;
            cout << "Matched Inliers" <<  mnMatchedInlier << endl;
            //if(mbCurrentFrameFeaturesReady)
            if(false)
            {
                cout << "Current Frame Features: " << endl;
                cout << "Number of Key Points:" << mnkeypoints << endl; // prints 10 decimals
                cout << "Avg Mappoint Depth:" << setprecision(5) << mfMapPointAvgMinDepth << endl; // prints 10 decimals
                cout << "Var Mappoint Depth:" << setprecision(5) << mfMapPointVarMinDepth << endl; // prints 10 decimals
            }
        }
        cout << endl;

        usleep(0.03*1000*1000);
    }
}

//////////////////////////////////////////////////////////////////////////
// TODO: Define methods to collect features from ORB-SLAM3
//////////////////////////////////////////////////////////////////////////

//void DataCollecting::DefineMethod2CollectFeatures()
//{
//    // TODO
//}

void DataCollecting::CollectImageTimeStamp(const double &timestamp)
{
    // TODO
    unique_lock<mutex> lock(mMutexImageTimeStamp);
    mTimeStamp = timestamp;
}

void DataCollecting::CollectImageFileName(string &filename)
{
    // TODO
    unique_lock<mutex> lock(mMutexImageFileName);
    msImgFileName = filename;
}

void DataCollecting::CollectImagePixel(cv::Mat &imGray)
{
    {
        unique_lock<mutex> lock1(mMutexImagePixel);
        //Option1: save the image with the same resolution
        //mImGrey = imGray.clone();

        //Option2: save the image with reduced resolution (reduced by 1/16 = 1/4*1/4)
        cv::resize(imGray, mImGrey, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
    }
    {
        unique_lock<mutex> lock2(mMutexImageCounter);
        mnImCounter ++;
    };

}


void DataCollecting::CollectCurrentFrame(const Frame &frame)
{
    unique_lock<mutex> lock(mMutexCurrentFrame);
    //mCurrentFrame = Frame(frame);
    mCurrentFrame = frame;
    mbCurrentFrameFeaturesReady = true;
}

void DataCollecting::CollectCurrentFrameTrackMode(const int &nTrackMode)
{
    // 0 for Track with Motion
    // 1 for Track with Reference Frame
    // 2 for Track with Relocalization
    unique_lock<mutex> lock(mMutexCurrentFrameTrackMode);
    mnTrackMode = nTrackMode;
}

void DataCollecting::CollectCurrentFramePrePOOutlier(const int &nPrePOOutlier)
{
    unique_lock<mutex> lock(mMutexCurrentFramePrePOOutlier);
    mnPrePOOutlier = nPrePOOutlier;
}

void DataCollecting::CollectCurrentFramePrePOKeyMapLoss(const int &nPrePOKeyMapLoss)
{
    unique_lock<mutex> lock(mMutexCurrentFramePrePOKeyMapLoss);
    mnPrePOKeyMapLoss = nPrePOKeyMapLoss;
}

void DataCollecting::CollectCurrentFrameInlier(const int &nInlier)
{
    unique_lock<mutex> lock(mMutexCurrentFrameInlier);
    mnInlier = nInlier;
}

void DataCollecting::CollectCurrentFramePostPOOutlier(const int &nPostPOOutlier)
{
    unique_lock<mutex> lock(mMutexCurrentFramePostPOOutlier);
    mnPostPOOutlier = nPostPOOutlier;
}

void DataCollecting::CollectCurrentFramePostPOKeyMapLoss(const int &nPostPOKeyMapLoss)
{
    unique_lock<mutex> lock(mMutexCurrentFramePostPOKeyMapLoss);
    mnPostPOKeyMapLoss = nPostPOKeyMapLoss;
}

void DataCollecting::CollectCurrentFrameMatchedInlier(const int &nMatchedInlier)
{
    unique_lock<mutex> lock(mMutexCurrentFrameMatchedInlier);
    mnMatchedInlier = nMatchedInlier;
}


//////////////////////////////////////////////////////////////////////////
// END: Methods to collect features from ORB-SLAM3
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// TODO: Methods to process features from ORB-SLAM3
//////////////////////////////////////////////////////////////////////////

double DataCollecting::CalculateImageEntropy(const cv::Mat& image)
{
    // Calculate histogram of pixel intensities
    cv::Mat hist;
    int histSize = 256;  // Number of bins for intensity values
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    // Normalize histogram
    hist /= image.total();

    // Calculate entropy
    double entropy = 0;
    for (int i = 0; i < histSize; ++i)
    {
        if (hist.at<float>(i) > 0)
        {
            entropy -= hist.at<float>(i) * std::log2(hist.at<float>(i));
        }
    }
    return entropy;
}

void DataCollecting::CalculateImageFeatures()
{
    unique_lock<mutex> lock1(mMutexImagePixel);
    unique_lock<mutex> lock2(mMutexImageStatistics);
    if (mImGrey.empty())
    {
        cout << "Failed to load image." << endl;
    }
    else
    {
        cv::Scalar meanValue, stddevValue;
        // calculate the brightness and contrast
        cv::meanStdDev(mImGrey, meanValue, stddevValue);
        mdBrightness = meanValue[0];
        mdContrast = stddevValue[0];
        // calculate the entropy
        mdEntropy = CalculateImageEntropy(mImGrey);
        // update the data ready flag
        mbImageFeaturesReady = true;
    }

}


float DataCollecting::CalculateMapPointDepth(Frame frame)
{
    vector<float> vMapPointMinDepth;
    for (int i = 0; i < frame.N; i++)
    {
        // If current frame i-th key point is not Null  --> it is also a map point
        if (mCurrentFrame.mvpMapPoints[i])
        {
            // cout << "Mean - MinDepth: " << mCurrentFrame.mvpMapPoints[i]->GetMinDistanceInvariance() << endl;
            vMapPointMinDepth.push_back(mCurrentFrame.mvpMapPoints[i]->GetMinDistanceInvariance());
            //vMapPointMaxDepth.push_back(mCurrentFrame.mvpMapPoints[i]->GetMaxDistanceInvariance());
        }
    }
    // Calculate the mean
    mfMapPointAvgMinDepth = accumulate(vMapPointMinDepth.begin(), vMapPointMinDepth.end(), 0.0) / vMapPointMinDepth.size();
    // Calculate the variance
    float sum_of_squares = 0;
    for (auto num : vMapPointMinDepth) 
    {
        sum_of_squares += pow(num - mfMapPointAvgMinDepth, 2);
    }
    mfMapPointVarMinDepth = sum_of_squares / vMapPointMinDepth.size();
}


void DataCollecting::CalculateCurrentFrameFeatures()
{
    unique_lock<mutex> lock1(mMutexCurrentFrame);
    if(mbCurrentFrameFeaturesReady)
    {
        unique_lock<mutex> lock2(mMutexCurrentFrameFeatures);
        // TODO
        // 1. number of keypoints
        mnkeypoints = mCurrentFrame.N;
        // 2. average mappoint depth
        // 3. variance of the mappoint depth
        CalculateMapPointDepth(mCurrentFrame);
    }
}

//void DataCollecting::CollectFrameLaplacian()
//{
//    // TODO
//}
//
//void DataCollecting::CollectFrameOriginalKeypointNumber()
//{
//    // TODO
//    // Find the number of keypoints with the initial threshold
//}
//
//void DataCollecting::CollectFrameFinalKeypointNumber()
//{
//    // TODO
//    // Find the number of keypoints with the final threshold
//}
//
//void DataCollecting::CollectFrameFinalKeypointDistribution()
//{
//    // TODO
//}
//
//void DataCollecting::CollectInliers()
//{
//    // TODO
//}
//
//void DataCollecting::CollectLocalBAVisualError()
//{
//    // Collect the bundle adjustment error of visual input in local mapping module
//    // TODO
//}
//
//void DataCollecting::CollectLocalBAInertialError()
//{
//    // Collect the bundle adjustment error of inertial input in local mapping module
//    // TODO
//}
//
//void DataCollecting::CollectPoseAbsolutefPosition()
//{
//    // TODO
//}
//
//void DataCollecting::CollectPoseAbsoluteRotation()
//{
//    // TODO
//}


//////////////////////////////////////////////////////////////////////////
// END: Methods to process features from ORB-SLAM3
//////////////////////////////////////////////////////////////////////////


void DataCollecting::Release()
{
    // TODO
}


// End of the ORBSLAM3 namespace
}
