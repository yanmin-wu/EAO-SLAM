#ifndef __SFMTRANSCRIPTINTERFACE_ORBSLAM_H
#define __SFMTRANSCRIPTINTERFACE_ORBSLAM_H

#include "CARV/SFMTranscript.h"
#include <string>
#include <set>
#include <map>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {
    class KeyFrame;
    class MapPoint;
}
class Cluster;

class SFMTranscriptInterface_ORBSLAM;
class SFMTranscriptInterface_ORBSLAM{
public:
    // Constructors and Destructors
    SFMTranscriptInterface_ORBSLAM();
    ~SFMTranscriptInterface_ORBSLAM();

    // Getters
    dlovi::compvis::SFMTranscript * getTranscriptRef();
    dlovi::compvis::SFMTranscript * getTranscriptToProcessRef();

    // Public Methods
    void addResetEntry();
    void addPointDeletionEntry(ORB_SLAM2::MapPoint *p);
    void addVisibilityRayInsertionEntry(ORB_SLAM2::KeyFrame *k, ORB_SLAM2::MapPoint *p);
    void addVisibilityRayDeletionEntry(ORB_SLAM2::KeyFrame *k, ORB_SLAM2::MapPoint *p);
    void addFirstKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *k);
    void addKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *k);

    void addSemiDenseKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *k);
    void addLineSegmentInsertionEntry(ORB_SLAM2::KeyFrame *kf, Cluster *pCluster);
    void addLineSegmentKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *kf);

    void addBundleAdjustmentEntry(std::set<ORB_SLAM2::KeyFrame *> & sAdjustSet, std::set<ORB_SLAM2::MapPoint *> & sMapPoints);
    void writeToFile(const std::string & strFileName) const;
    void suppressBundleAdjustmentLogging();
    void unsuppressBundleAdjustmentLogging();
    void suppressRefindLogging();
    void unsuppressRefindLogging();

    void UpdateTranscriptToProcess();

    std::vector<ORB_SLAM2::MapPoint *> GetNewPoints(ORB_SLAM2::KeyFrame *pKF);

private:
    // Member Variables
    dlovi::compvis::SFMTranscript m_SFMTranscript;

    // transcript that is guarded by mutex
    dlovi::compvis::SFMTranscript m_SFMTranscriptToProcess;

    bool m_bSuppressRefindLogging;
    bool m_bSuppressBundleAdjustmentLogging;

    std::map<ORB_SLAM2::MapPoint *, int> m_mMapPoint_Index;	// TODO: possibly change to hashed?
    std::map<ORB_SLAM2::KeyFrame *, int> m_mKeyFrame_Index;	// TODO: possibly change to hashed?
    // Store ID instead of pointer
    std::map<long unsigned int, int> m_mFrame_Index;

    // correspondence between added keyframes and mappoints
    std::map<ORB_SLAM2::KeyFrame *, std::vector<ORB_SLAM2::MapPoint *>> m_mKeyFrame_MapPoint;
};

#endif
