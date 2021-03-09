
#ifndef __MODELER_H
#define __MODELER_H

#include <mutex>

#include <list>
#include <vector>
#include "CARV/Matrix.h"
#include "CARV/SFMTranscriptInterface_ORBSLAM.h"
#include "CARV/SFMTranscriptInterface_Delaunay.h"

class Cluster;

namespace ORB_SLAM2 {
    class KeyFrame;
    class Map;
    class MapPoint;
}


class Model {
public:
    Model(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris);
    vector<dlovi::Matrix> & GetPoints();
    list<dlovi::Matrix> & GetTris();

    void SetNotErase();
    void SetErase();
    void Release();

private:
    std::mutex mMutexErase;
    bool mbNotErase;
    bool mbToBeErased;
    std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix>> mData;

};


// interface class for surface reconstruction using CARV system
class Modeler {
public:
    Modeler(ORB_SLAM2::Map* pMap);

    void AddKeyFrameEntry(ORB_SLAM2::KeyFrame* pKF);
    void AddLineSegmentKeyFrameEntry(ORB_SLAM2::KeyFrame* pKF);

    bool CheckNewTranscriptEntry();

    void RunRemainder();
    void RunOnce();
    void UpdateModel();

    void WriteModel(std::string filename);

public:
    //CARV interface
    SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
    //CARV runner instance
    dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
    SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.

    ORB_SLAM2::Map* mpMap;

    // This avoid that two transcript entries are created simultaneously in separate threads
    std::mutex mMutexTranscript;

    //number of lines in transcript last time checked
    int mnLastNumLines;

    bool mbFirstKeyFrame;

};



#endif //__MODELVIEWER_H
