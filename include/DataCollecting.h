//
// Created by tianyi on 6/15/23.
//

#ifndef ORB_SLAM3_DATACOLLECTING_H
#define ORB_SLAM3_DATACOLLECTING_H

#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

namespace  ORB_SLAM3
{
class System;
class Tracking;
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

};


}


#endif //ORB_SLAM3_DATACOLLECTING_H
