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
    //mbMonocular = bMonocular;
    //mbInertial = bInertial;
    //mpSystem = pSys;
    mpAtlas = pAtlas;
    msSeqName = _strSeqName;

    mbFinished = false;
    mbImageFeaturesReady = false;
    mbCurrentFrameFeaturesReady = false;
    mnImCounter = 0;
    mbIsNewFrameProcessed = true; // Initialized as it is already processed

    InitializeCSVLogger();

    cout << "Data Collecting Module Initialized" << endl;

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

void DataCollecting::Run()
{
    // Initialize the data collector
    cout << "Data Collecting Module Running" << endl;

    usleep(0.1*1000*1000);

    while(1)
    {
        //TODO
        if(!mbIsNewFrameProcessed)
        {
            auto start = chrono::high_resolution_clock::now();
            
            // If the new arrived frame haven't been processed
            if(mbImageFeaturesReady)
            {
                CalculateImageFeatures();
            }
            if(mbCurrentFrameFeaturesReady)
            {
                CalculateCurrentFrameFeatures();
            }

            if (mbImageFeaturesReady && mbCurrentFrameFeaturesReady)
            {
                WriteRowCSVLogger();
            }

            {
                unique_lock<mutex> lock(mMutexNewFrameProcessed);
                mbIsNewFrameProcessed = true;
            }

            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
            cout << "Run() elapsed: " << duration.count() << " microseconds" << endl;

        }



//        cout << "Testing Data Collection." << endl;
//        {
//            unique_lock<mutex> lock(mMutexImageTimeStamp);
//            cout << "Current Time Stamp: " << setprecision(10) << mTimeStamp << endl; // prints 10 decimals
//        }
//        {
//            unique_lock<mutex> lock(mMutexImageCounter);
//            cout << "Current Image Counter: " << mnImCounter << endl; //
//        }
//
//        {
//            unique_lock<mutex> lock(mMutexImageFeatures);
//            if(mbImageFeaturesReady)
//            {
//                cout << "Current Image Features: " << endl;
//                cout << "Brightness:" << setprecision(5) << mdBrightness << endl; // prints 10 decimals
//                cout << "Contrast:" << setprecision(5) << mdContrast << endl; // prints 10 decimals
//                cout << "Entropy:" << setprecision(5) << mdEntropy << endl; // prints 10 decimals
//            }
//        }
//
//        {
//            cout << "Current Frame Features: " << endl;
//            {
//                unique_lock<mutex> lock(mMutexCurrentFrameTrackMode);
//                cout << "Track Mode: " <<  mnTrackMode << endl;
//            }
//            {
//                unique_lock<mutex> lock(mMutexCurrentFrameInlier);
//                cout << "Matched Inliers: " <<  mnMatchedInlier << endl;
//            }
//        }
//        {
//            unique_lock<mutex> lock(mMutexCurrentFrameFeatures);
//            if(mbCurrentFrameFeaturesReady)
//            {
//                cout << "Current Frame Features: " << endl;
//                cout << "Number of Key Points:" << mnkeypoints << endl; // prints 10 decimals
//
//            }
//        }
//        {
//            unique_lock<mutex> lock(mMutexCurrentFrameMapPointDepth);
//            if(mbCurrentFrameFeaturesReady)
//            {
//                cout << "Avg Mappoint Depth:" << setprecision(5) << mfMapPointAvgMinDepth << endl; // prints 10 decimals
//                cout << "Var Mappoint Depth:" << setprecision(5) << mfMapPointVarMinDepth << endl; // prints 10 decimals
//            }
//
//        }
//        cout << endl;

        usleep(0.01*1000*1000);
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
    mdTimeStamp = timestamp;
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
        cv::resize(imGray, mImGrey, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
    }
    {
        unique_lock<mutex> lock2(mMutexImageCounter);
        mnImCounter ++;
    }
    // update the data ready flag
    mbImageFeaturesReady = true;
    {
        unique_lock<mutex> lock(mMutexNewFrameProcessed);
        mbIsNewFrameProcessed = false;
    }

}


void DataCollecting::CollectCurrentFrame(const Frame &frame)
{
    unique_lock<mutex> lock(mMutexCurrentFrame);
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

void DataCollecting::CollectCurrentFrameMapPointDepth(const Frame &frame)
{
    unique_lock<mutex> lock(mMutexCurrentFrameMapPointDepth);
    vector<float> vMapPointMinDepth;
    for (int i = 0; i < frame.N; i++)
    {
        // If current frame i-th map point is not Null
        if (frame.mvpMapPoints[i])
            vMapPointMinDepth.push_back(frame.mvpMapPoints[i]->GetMinDistanceInvariance());
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
    unique_lock<mutex> lock2(mMutexImageFeatures);
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
    }

}


void DataCollecting::CalculateCurrentFrameFeatures()
{
    unique_lock<mutex> lock1(mMutexCurrentFrame); 
    if(mbCurrentFrameFeaturesReady)
    {
        unique_lock<mutex> lock2(mMutexCurrentFrameFeatures);
        mnkeypoints = mCurrentFrame.N;
    }
}



//
//void DataCollecting::CollectFrameFinalKeypointDistribution()
//{
//    // TODO
//}
//


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


void DataCollecting::RequestFinish()
{
    // TODO
    mFileLogger.close();
    mbFinished = true;

}

//////////////////////////////////////////////////////////////////////////
// START: Methods to save features in .CSV
//////////////////////////////////////////////////////////////////////////

void DataCollecting::CollectCurrentTime()
{
    // Get the current time
    std::time_t currentTime = std::time(nullptr);

    // Convert the time to a string
    std::tm* now = std::localtime(&currentTime);

    // Format the date and time string
    std::ostringstream oss;
    oss << std::put_time(now, "%y%m%d_%H%M%S");
    msCurrentTime = oss.str();
}

void DataCollecting::InitializeCSVLogger()
{
    CollectCurrentTime();
    msCSVFileName = msSeqName + "_" + msCurrentTime + ".csv";

    // Open the CSV file for writing
    mFileLogger.open(msCSVFileName);

    // Write the first row with column names
    if (!mFileLogger.is_open()) 
    {
        std::cerr << "Unable to open file: " << msCSVFileName << std::endl;
    }
    else
    {
        // Write column names
        for (const auto& columnName : mvsColumnFeatureNames)
        {
            mFileLogger << columnName << ",";
        }
        mFileLogger << endl;
    }
}


void DataCollecting::WriteRowCSVLogger()
{
    if (!mFileLogger.is_open())
    {
        std::cerr << "Unable to open file: " << msCSVFileName << std::endl;
    }
    else
    {
        {
            unique_lock<mutex> lock(mMutexImageCounter);
            mFileLogger << mnImCounter << ", ";
        }
        {
            unique_lock<mutex> lock(mMutexImageTimeStamp);
            mFileLogger << fixed << setprecision(6) << 1e9*mdTimeStamp << ", ";
        }
        {
            unique_lock<mutex> lock(mMutexImageFileName);
            mFileLogger << msImgFileName;
        }

        if(mbImageFeaturesReady)
        {
            {
                unique_lock<mutex> lock(mMutexImageFeatures);
                mFileLogger << ", " << fixed << std::setprecision(6) << mdBrightness << ", " << mdContrast << ", " << mdEntropy;
            }
        }

        if(mbCurrentFrameFeaturesReady)
        {
            {
                unique_lock<mutex> lock(mMutexCurrentFrameMapPointDepth);
                mFileLogger  << ", " << mfMapPointAvgMinDepth << ", " << mfMapPointVarMinDepth << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFrameTrackMode);
                mFileLogger << mnTrackMode << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFramePrePOOutlier);
                mFileLogger << mnPrePOOutlier << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFramePrePOKeyMapLoss);
                mFileLogger << mnPrePOKeyMapLoss << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFrameInlier);
                mFileLogger << mnInlier << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFramePostPOOutlier);
                mFileLogger << mnPostPOOutlier << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFramePostPOKeyMapLoss);
                mFileLogger << mnPostPOKeyMapLoss << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFrameMatchedInlier);
                mFileLogger << mnMatchedInlier << ", ";
            }
            {
                unique_lock<mutex> lock(mMutexCurrentFrameFeatures);
                mFileLogger << mnkeypoints;
            }
        }
        
        mFileLogger << endl;
    }
}


// End of the ORBSLAM3 namespace
}


