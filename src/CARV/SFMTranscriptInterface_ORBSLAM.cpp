#ifndef __SFMTRANSCRIPTINTERFACE_ORBSLAM_CPP
#define __SFMTRANSCRIPTINTERFACE_ORBSLAM_CPP

#include "CARV/SFMTranscriptInterface_ORBSLAM.h"
#include "CARV/Exception.h"
#include "CARV/Matrix.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <mutex>
#include "LineDetector.h"

using namespace std;
using namespace dlovi;

// Constructors and Destructors

SFMTranscriptInterface_ORBSLAM::SFMTranscriptInterface_ORBSLAM(){
    try{
        // No suppression of logging by default
        m_bSuppressBundleAdjustmentLogging = false;
        m_bSuppressRefindLogging = false;

        // Write transcript header
        m_SFMTranscript.addLine("SFM Transcript: ORBSLAM");
        m_SFMTranscript.addLine("*** BODY ***");
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "SFMTranscriptInterface_ORBSLAM"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

SFMTranscriptInterface_ORBSLAM::~SFMTranscriptInterface_ORBSLAM(){
    // NOP
}

// Getters
dlovi::compvis::SFMTranscript * SFMTranscriptInterface_ORBSLAM::getTranscriptRef(){
    try{
        return & m_SFMTranscript;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "getTranscriptRef"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

dlovi::compvis::SFMTranscript * SFMTranscriptInterface_ORBSLAM::getTranscriptToProcessRef(){
    try{
        return & m_SFMTranscriptToProcess;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "getTranscriptToProcessRef"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

// Public Methods

void SFMTranscriptInterface_ORBSLAM::addResetEntry(){
    try{
        m_SFMTranscript.addLine("reset");
        // Reset the pointer -> index maps
        m_mMapPoint_Index.clear();
        m_mKeyFrame_Index.clear();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addResetEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::addPointDeletionEntry(ORB_SLAM2::MapPoint *p){
    try{
        std::stringstream ssTmp;

        // Set nPointIndex based on argument.
        std::map<ORB_SLAM2::MapPoint *, int>::iterator itMapPoint = m_mMapPoint_Index.find(p);
        if(itMapPoint == m_mMapPoint_Index.end()) // The logger has no record of this point?  That's bad.
        {
            return;
        }
        int nPointIndex = itMapPoint->second;

        ssTmp << "del point: " << nPointIndex;
        m_SFMTranscript.addLine(ssTmp.str());
    }

    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addPointDeletionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::addVisibilityRayInsertionEntry(ORB_SLAM2::KeyFrame *k, ORB_SLAM2::MapPoint *p){
    try{
        if(! m_bSuppressRefindLogging){
            std::stringstream ssTmp;

            // Set nCamIndex and nPointIndex based on arguments.
            std::map<ORB_SLAM2::KeyFrame *, int>::iterator itKeyFrame = m_mKeyFrame_Index.find(k);
            if(itKeyFrame == m_mKeyFrame_Index.end()) // The logger has no record of this KF?  That's bad.
                throw dlovi::Exception("Could not compute KeyFrame index: no record.");
            std::map<ORB_SLAM2::MapPoint *, int>::iterator itMapPoint = m_mMapPoint_Index.find(p);
            if(itMapPoint == m_mMapPoint_Index.end()) // The logger has no record of this point?  That's bad.
                throw dlovi::Exception("Could not compute MapPoint index: no record.");

            int nCamIndex = itKeyFrame->second;
            int nPointIndex = itMapPoint->second;

            ssTmp << "observation: " << nCamIndex << ", " << nPointIndex;
            m_SFMTranscript.addLine(ssTmp.str());
        }
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addVisibilityRayInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
        assert(0); // TODO: remove me
    }
}

void SFMTranscriptInterface_ORBSLAM::addVisibilityRayDeletionEntry(ORB_SLAM2::KeyFrame *k, ORB_SLAM2::MapPoint *p){
    try{
        std::stringstream ssTmp;

        // Set nCamIndex and nPointIndex based on arguments.
        std::map<ORB_SLAM2::KeyFrame *, int>::iterator itKeyFrame = m_mKeyFrame_Index.find(k);
        if(itKeyFrame == m_mKeyFrame_Index.end()) // The logger has no record of this KF?  That's bad.
        {
            return;
        }
        std::map<ORB_SLAM2::MapPoint *, int>::iterator itMapPoint = m_mMapPoint_Index.find(p);
        if(itMapPoint == m_mMapPoint_Index.end()) // The logger has no record of this point?  That's bad.
        {
            return;
        }

        int nCamIndex = itKeyFrame->second;
        int nPointIndex = itMapPoint->second;

        ssTmp << "del observation: " << nCamIndex << ", " << nPointIndex;
        m_SFMTranscript.addLine(ssTmp.str());
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addVisibilityRayDeletionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}


void SFMTranscriptInterface_ORBSLAM::addLineSegmentInsertionEntry(ORB_SLAM2::KeyFrame *kf, Cluster *pCluster){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matClusterS(3, 1);
        dlovi::Matrix matClusterE(3, 1);
        dlovi::Matrix matNewPointS(3, 1);
        dlovi::Matrix matNewPointE(3, 1);

        int nCamIndex, nClusterIndex;

        if(m_mKeyFrame_Index.count(kf) <= 0)
            throw dlovi::Exception("KeyFrame does not have a record.");

        nCamIndex = m_mKeyFrame_Index[kf];
        nClusterIndex = (int)(pCluster->index);

        matClusterS(0) = pCluster->centerSegment.at<float>(0,0);
        matClusterS(1) = pCluster->centerSegment.at<float>(0,1);
        matClusterS(2) = pCluster->centerSegment.at<float>(0,2);
        matClusterE(0) = pCluster->centerSegment.at<float>(0,3);
        matClusterE(1) = pCluster->centerSegment.at<float>(0,4);
        matClusterE(2) = pCluster->centerSegment.at<float>(0,5);

        int i = pCluster->segments.rows - 1;
        matNewPointS(0) = pCluster->segments.at<float>(i,0);
        matNewPointS(1) = pCluster->segments.at<float>(i,1);
        matNewPointS(2) = pCluster->segments.at<float>(i,2);
        matNewPointE(0) = pCluster->segments.at<float>(i,3);
        matNewPointE(1) = pCluster->segments.at<float>(i,4);
        matNewPointE(2) = pCluster->segments.at<float>(i,5);

        ssTmp << "new line: [" << matNewPointS(0) << "; " << matNewPointS(1) << "; " << matNewPointS(2) << "]";
        ssTmp << ", [" << matNewPointE(0) << "; " << matNewPointE(1) << "; " << matNewPointE(2) << "]";
        ssTmp << ", " << nClusterIndex;
        ssTmp << ", [" << matClusterS(0) << "; " << matClusterS(1) << "; " << matClusterS(2) << "]";
        ssTmp << ", [" << matClusterE(0) << "; " << matClusterE(1) << "; " << matClusterE(2) << "]";
        ssTmp << ", " << nCamIndex;

        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addLineEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }

}

void SFMTranscriptInterface_ORBSLAM::addLineSegmentKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *kf){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matNewCam(3, 1);
        dlovi::Matrix matNewPointS(3, 1);
        dlovi::Matrix matNewPointE(3, 1);
        dlovi::Matrix matNewPointM(3, 1);

        int nCamIndex;

        if(m_mKeyFrame_Index.count(kf) > 0)
            throw dlovi::Exception("KeyFrame already has a record.  Double addition.");

        // GetPoseInverse, seems camera position need to be inversed
        cv::Mat se3WfromC = kf->GetPoseInverse();
        matNewCam(0) = se3WfromC.at<float>(0,3);
        matNewCam(1) = se3WfromC.at<float>(1,3);
        matNewCam(2) = se3WfromC.at<float>(2,3);
        ssTmp << "new cam: [" << matNewCam(0) << "; " << matNewCam(1) << "; " << matNewCam(2) << "] {";
        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        // Add a record of the new camera to internal map.
        nCamIndex = m_mKeyFrame_Index.size();
        m_mKeyFrame_Index[kf] = nCamIndex;

        for (int i = 0; i < kf->mLines3D.rows; i++) {
            matNewPointS(0) = kf->mLines3D.at<float>(i,0);
            matNewPointS(1) = kf->mLines3D.at<float>(i,1);
            matNewPointS(2) = kf->mLines3D.at<float>(i,2);
            matNewPointE(0) = kf->mLines3D.at<float>(i,3);
            matNewPointE(1) = kf->mLines3D.at<float>(i,4);
            matNewPointE(2) = kf->mLines3D.at<float>(i,5);

            ssTmp << "new point: [" << matNewPointS(0) << "; " << matNewPointS(1) << "; " << matNewPointS(2) << "]";
            ssTmp << ", " << nCamIndex;
            m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

            ssTmp << "new point: [" << matNewPointE(0) << "; " << matNewPointE(1) << "; " << matNewPointE(2) << "]";
            ssTmp << ", " << nCamIndex;
            m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        }

        // Close this new-KF entry in the transcript
        m_SFMTranscript.addLine("}");

    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addFirstKeyFrameInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }

}

void SFMTranscriptInterface_ORBSLAM::addSemiDenseKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *kf){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matNewCam(3, 1);
        dlovi::Matrix matNewPoint(3, 1);
        int nCamIndex;

        if(m_mKeyFrame_Index.count(kf) > 0)
            throw dlovi::Exception("KeyFrame already has a record.  Double addition.");

        // GetPoseInverse, seems camera position need to be inversed
        cv::Mat se3WfromC = kf->GetPoseInverse();
        matNewCam(0) = se3WfromC.at<float>(0,3);
        matNewCam(1) = se3WfromC.at<float>(1,3);
        matNewCam(2) = se3WfromC.at<float>(2,3);
        ssTmp << "new cam: [" << matNewCam(0) << "; " << matNewCam(1) << "; " << matNewCam(2) << "] {";
        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        // Add a record of the new camera to internal map.
        nCamIndex = m_mKeyFrame_Index.size();
        m_mKeyFrame_Index[kf] = nCamIndex;

        // access semi dense points with lock
        {
            std::unique_lock<std::mutex> lock(kf->mMutexSemiDensePoints);

            for (size_t y = 0; y < (size_t)kf->im_.rows; y++)
                for (size_t x = 0; x < (size_t)kf->im_.cols; x++) {
                    if (kf->depth_sigma_.at<float>(y, x) > 0.02) continue;

                    if (kf->depth_map_checked_.at<float>(y, x) > 0.000001) {

                        matNewPoint(0) = kf->SemiDensePointSets_.at<float>(y, 3 * x);
                        matNewPoint(1) = kf->SemiDensePointSets_.at<float>(y, 3 * x + 1);
                        matNewPoint(2) = kf->SemiDensePointSets_.at<float>(y, 3 * x + 2);

                        ssTmp << "new point: [" << matNewPoint(0) << "; " << matNewPoint(1) << "; " << matNewPoint(2) << "]";
                        ssTmp << ", " << nCamIndex;
                        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

                    }
                }

        }
        // Close this new-KF entry in the transcript
        m_SFMTranscript.addLine("}");

    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addFirstKeyFrameInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }

}

void SFMTranscriptInterface_ORBSLAM::addFirstKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *k){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matNewCam(3, 1);
        dlovi::Matrix matNewPoint(3, 1);
        int nPointIndex, nCamIndex;

        if(m_mKeyFrame_Index.count(k) > 0)
            throw dlovi::Exception("KeyFrame already has a record.  Double addition.");

        // // TODO: Instead of inverting the whole transform, we should be able to just use the negative translation.
        // GetPoseInverse, seems camera position need to be inversed
        cv::Mat se3WfromC = k->GetPoseInverse();
        matNewCam(0) = se3WfromC.at<float>(0,3);
        matNewCam(1) = se3WfromC.at<float>(1,3);
        matNewCam(2) = se3WfromC.at<float>(2,3);
        ssTmp << "new cam: [" << matNewCam(0) << "; " << matNewCam(1) << "; " << matNewCam(2) << "] {";
        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        // Add a record of the new camera to internal map.
        nCamIndex = m_mKeyFrame_Index.size();
        m_mKeyFrame_Index[k] = nCamIndex;

        // Add the keyframe to the coorespondence map
        vector<ORB_SLAM2::MapPoint *> vMP;
        m_mKeyFrame_MapPoint[k] = vMP;

        std::set<ORB_SLAM2::MapPoint*> mvpMapPoints = k->GetMapPoints();
        // Process new points and visibility information in this KF
        for(std::set<ORB_SLAM2::MapPoint*>::iterator it = mvpMapPoints.begin(); it != mvpMapPoints.end(); it++){
            ORB_SLAM2::MapPoint * point = *it;
            if(point->isBad())
                continue;
            if(m_mMapPoint_Index.count(point) == 0){
                // It's a new point:
                cv::Mat mWorldPos = point->GetWorldPos();
                matNewPoint(0) = mWorldPos.at<float>(0);
                matNewPoint(1) = mWorldPos.at<float>(1);
                matNewPoint(2) = mWorldPos.at<float>(2);

                ssTmp << "new point: [" << matNewPoint(0) << "; " << matNewPoint(1) << "; " << matNewPoint(2) << "]";
                // Append this point's vis list with special handling.  (Point initialized from epipolar search: > 1 KF, but only 1 KF in our internal structures.)
                ssTmp << ", 0"; // KF 0 observed it.
                m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

                // Add a record of the new point to internal map.
                nPointIndex = m_mMapPoint_Index.size();
                m_mMapPoint_Index[point] = nPointIndex;

                // Add the correspondence to map
                m_mKeyFrame_MapPoint[k].push_back(point);

            }
            else
                throw dlovi::Exception("The FIRST KF observed a point that was already added."); // That's bad!  Points are only added through KF-addition.
        }

        // Close this new-KF entry in the transcript
        m_SFMTranscript.addLine("}");
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addFirstKeyFrameInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}


void SFMTranscriptInterface_ORBSLAM::addKeyFrameInsertionEntry(ORB_SLAM2::KeyFrame *k){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matNewCam(3, 1);
        dlovi::Matrix matNewPoint(3, 1);

        // points on line segments
        dlovi::Matrix matNewPointS(3, 1);
        dlovi::Matrix matNewPointE(3, 1);
        dlovi::Matrix matNewPointM(3, 1);

        int nPointIndex, nCamIndex;

        if(m_mKeyFrame_Index.count(k) > 0)
            throw dlovi::Exception("KeyFrame already has a record.  Double addition.");

        // TODO: Instead of inverting the whole transform, we should be able to just use the negative translation.
        // GetPoseInverse, seems camera position need to be inversed
        cv::Mat se3WfromC = k->GetPoseInverse();
        matNewCam(0) = se3WfromC.at<float>(0,3);
        matNewCam(1) = se3WfromC.at<float>(1,3);
        matNewCam(2) = se3WfromC.at<float>(2,3);
        ssTmp << "new cam: [" << matNewCam(0) << "; " << matNewCam(1) << "; " << matNewCam(2) << "] {";
        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        // Add a record of the new camera to internal map.
        nCamIndex = m_mKeyFrame_Index.size();
        m_mKeyFrame_Index[k] = nCamIndex;

        // Add the keyframe to the coorespondence map
        vector<ORB_SLAM2::MapPoint *> vMP;
        m_mKeyFrame_MapPoint[k] = vMP;

        // Process new points and visibility information in this KF
        std::set<int> sVisListExcludingNewPoints;
        std::set<ORB_SLAM2::MapPoint*> mvpMapPoints = k->GetMapPoints();
        for(std::set<ORB_SLAM2::MapPoint*>::iterator it = mvpMapPoints.begin(); it != mvpMapPoints.end(); it++){
            ORB_SLAM2::MapPoint * point = *it;
            if(point->isBad())
                continue;
            if(m_mMapPoint_Index.count(point) == 0){

                // check if it has valid observation first
                std::map<ORB_SLAM2::KeyFrame*, size_t> mObservations = point->GetObservations();

                bool hasObservation = false;
                for(std::map<ORB_SLAM2::KeyFrame *,size_t>::iterator it2 = mObservations.begin(); it2 != mObservations.end(); it2++){
                    if(m_mKeyFrame_Index.count(it2->first) != 0) {
                        hasObservation = true;
                        break;
                    }
                }

                // It's a new point:
                cv::Mat mWorldPos = point->GetWorldPos();
                matNewPoint(0) = mWorldPos.at<float>(0);
                matNewPoint(1) = mWorldPos.at<float>(1);
                matNewPoint(2) = mWorldPos.at<float>(2);

                ssTmp << "new point: [" << matNewPoint(0) << "; " << matNewPoint(1) << "; " << matNewPoint(2)
                      << "]";

                if (hasObservation) {
                    // Append this point's vis list.  (Point initialized from epipolar search: > 1 KF)
                    for (std::map<ORB_SLAM2::KeyFrame *, size_t>::iterator it2 = mObservations.begin();
                         it2 != mObservations.end(); it2++) {
                        if (m_mKeyFrame_Index.count(it2->first) != 0) {
                            ssTmp << ", " << m_mKeyFrame_Index[it2->first];
                        }
                    }
                } else {
                    ssTmp << ", " << nCamIndex;
                }

                m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

                // Add a record of the new point to internal map.
                nPointIndex = m_mMapPoint_Index.size();
                m_mMapPoint_Index[point] = nPointIndex;

                // Add the correspondence to map
                m_mKeyFrame_MapPoint[k].push_back(point);

            }
            else{
                // It's not a new point:
                sVisListExcludingNewPoints.insert(m_mMapPoint_Index[point]); // To be added after the new points in the following loop
            }
        }

        // Log all the visibility-ray observations for this KF excluding the newly added points
        for(std::set<int>::iterator it = sVisListExcludingNewPoints.begin(); it != sVisListExcludingNewPoints.end(); it++){
            ssTmp << "observation: " << *it;
            m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        }

        // Close this new-KF entry in the transcript
        m_SFMTranscript.addLine("}");
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addKeyFrameInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}


void SFMTranscriptInterface_ORBSLAM::addBundleAdjustmentEntry(set<ORB_SLAM2::KeyFrame *> & sAdjustSet, set<ORB_SLAM2::MapPoint *> & sMapPoints){
    try{
        if(! m_bSuppressBundleAdjustmentLogging){
            std::stringstream ssTmp;
            int nPointIndex, nCamIndex;

            // TODO: stub

            m_SFMTranscript.addLine("bundle {");

            // Log point-move entries
            for(set<ORB_SLAM2::MapPoint *>::iterator it = sMapPoints.begin(); it != sMapPoints.end(); it++){
                if(m_mMapPoint_Index.count(*it) == 0)
                    continue;
                nPointIndex = m_mMapPoint_Index[*it];
                ssTmp << "move point: " << nPointIndex << ", [" << (*it)->GetWorldPos().at<float>(0) << "; " << (*it)->GetWorldPos().at<float>(1)
                      << "; " << (*it)->GetWorldPos().at<float>(2) << "]";
                m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");
            }

            // Log KF-move entries
            for(set<ORB_SLAM2::KeyFrame *>::iterator it = sAdjustSet.begin(); it != sAdjustSet.end(); it++){
                if(m_mKeyFrame_Index.count(*it) == 0)
                    continue;
                nCamIndex = m_mKeyFrame_Index[*it];

                // TODO: Instead of inverting the whole transform, we should be able to just use the negative translation.
                cv::Mat se3WfromC = (*it)->GetPose();
                se3WfromC = se3WfromC.inv();
                ssTmp << "move cam: " << nCamIndex << ", [" << se3WfromC.at<float>(0,3) << "; " << se3WfromC.at<float>(1,3) << "; "
                      << se3WfromC.at<float>(2,3) << "]";
                m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");
            }

            // Close this bundle-adjust entry in the transcript
            m_SFMTranscript.addLine("}");
        }
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addBundleAdjustmentEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::writeToFile(const std::string & strFileName) const{
    try{
        m_SFMTranscript.writeToFile(strFileName);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "writeToFile"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::suppressBundleAdjustmentLogging(){
    try{
        m_bSuppressBundleAdjustmentLogging = true;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "suppressBundleAdjustmentLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::unsuppressBundleAdjustmentLogging(){
    try{
        m_bSuppressBundleAdjustmentLogging = false;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "unsuppressBundleAdjustmentLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::suppressRefindLogging(){
    try{
        m_bSuppressRefindLogging = true;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "suppressRefindLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::unsuppressRefindLogging(){
    try{
        m_bSuppressRefindLogging = false;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "unsuppressRefindLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}


void SFMTranscriptInterface_ORBSLAM::UpdateTranscriptToProcess(){
    try{
        int numToProcess = m_SFMTranscriptToProcess.numLines();
        int numUpdated = m_SFMTranscript.numLines();
        for (int i = numToProcess; i < numUpdated; i++){
            m_SFMTranscriptToProcess.addLine(m_SFMTranscript.getLine(i));
        }
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "UpdateTranscriptToProcess"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

std::vector<ORB_SLAM2::MapPoint *> SFMTranscriptInterface_ORBSLAM::GetNewPoints(ORB_SLAM2::KeyFrame *pKF) {
    try{
        return m_mKeyFrame_MapPoint[pKF];
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "GetReferenceKeyFrame"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

#endif
