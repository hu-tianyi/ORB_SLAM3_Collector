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
    mbFinished = false;

    while(1)
    {
        //TODO

    }

}

void DataCollecting::Release()
{
    // TODO
}


// End of the ORBSLAM3 namespace
}