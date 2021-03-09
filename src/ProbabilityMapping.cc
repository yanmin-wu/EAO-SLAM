/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.cc
 *
 *    Description:
 *
 *        Version:  0.1
 *        Created:  01/21/2016 10:39:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Josh Tang, Rebecca Frederick
 *
 *        version: 1.0
 *        created: 8/9/2016
 *        Log: fix a lot of bug, Almost rewrite the code.
 *
 *        author: He Yijia
 * *
 *        Version: 1.1
 *        Created: 05/18/2017
 *        Author: Shida He
 *
 * =====================================================================================
 */

#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <random>
#include "ProbabilityMapping.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "LocalMapping.h"
#include "Modeler.h"


//#define OnlineLoop  // uncomment for online line segment and surface reconstruction
//#define ForceRealTime  //if uncommented, main tracking thread will wait for this thread to finish before processing next frame


template<typename T>
float bilinear(const cv::Mat& img, const float& y, const float& x)
{
    int x0 = (int)std::floor(x);
    int y0 = (int )std::floor(y);
    int x1 = x0 + 1;
    int y1 =  y0 + 1;

    float x0_weight = x1 - x;
    float y0_weight = y1 - y;
    float x1_weight = 1.0f - x0_weight;
    float y1_weight = 1.0f - y0_weight;
    float interpolated = img.at<T>(y0,x0) * x0_weight * y0_weight +
                         img.at<T>(y0,x1) * x1_weight * y0_weight +
                         img.at<T>(y1,x0) * x0_weight * y1_weight +
                         img.at<T>(y1,x1) * x1_weight * y1_weight;

    return interpolated;
}

template<typename T>
float ylinear(const cv::Mat& img, const float& y, const float& x)
{
    int x0 = (int)std::floor(x);
    assert((float)x0 == x);
    int y0 = (int)std::floor(y);
    int y1 =  y0 + 1;

    float y0_weight = y1 - y;
    float y1_weight = y - y0;

    float interpolated = img.at<T>(y0,x0) * y0_weight +
                         img.at<T>(y1,x0) * y1_weight;

    return interpolated;
}

template<typename T>
float yangle(const cv::Mat& img, const float& y, const float& x)
{
    int x0 = (int)std::floor(x);
    assert((float)x0 == x);
    int y0 = (int)std::floor(y);
    int y1 =  y0 + 1;

    float y0_weight = y1 - y;
    float y1_weight = y - y0;

    float a0 = img.at<T>(y0,x0);
    float a1 = img.at<T>(y1,x0);

    if (abs(a0-a1) < 180){
        return a0 * y0_weight + a1 * y1_weight;
    }else{
        if(a0 < a1){
            a0 += 360;
        } else {
            a1 += 360;
        }
        float inter = a0 * y0_weight + a1 * y1_weight;
        if (inter >= 360){
            inter -= 360;
        }
        return inter;
    }
}

Modeler* ProbabilityMapping::GetModeler()
{
    return mpModeler;
}


void ProbabilityMapping::WriteModel()
{
    boost::filesystem::path results_dir("results_line_segments/" +mLineDetector.GetStringDateTime());
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    std::string strFileName("results_line_segments/" + mLineDetector.GetStringDateTime() + "/model.obj");

    mpModeler->WriteModel(strFileName);
    std::cout << "saved mesh model" << std::endl;

}

void ProbabilityMapping::SaveSemiDensePoints()
{
    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    boost::filesystem::path results_dir("results_line_segments/" +mLineDetector.GetStringDateTime());
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    std::string strFileName("results_line_segments/" + mLineDetector.GetStringDateTime() + "/semi_pointcloud.obj");

    std::ofstream fileOut(strFileName.c_str(), std::ios::out);
    if(!fileOut){
        std::cerr << "Failed to save semi dense points" << std::endl;
        return;
    }

    for (size_t indKF = 0; indKF < vpKFs.size(); indKF++) {
        ORB_SLAM2::KeyFrame* kf = vpKFs[indKF];
        kf->SetNotEraseSemiDense();
        if( kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
            kf->SetEraseSemiDense();
            continue;
        }

        for(size_t y = 0; y< (size_t)kf->im_.rows; y++) {
            for (size_t x = 0; x < (size_t) kf->im_.cols; x++) {

                if (kf->depth_sigma_.at<float>(y,x) > 0.02) continue;
                if (kf->depth_map_checked_.at<float>(y,x) > 0.000001) {

                    Eigen::Vector3f Pw(kf->SemiDensePointSets_.at<float>(y, 3 * x),
                                       kf->SemiDensePointSets_.at<float>(y, 3 * x + 1),
                                       kf->SemiDensePointSets_.at<float>(y, 3 * x + 2));
                    float vr, vg, vb;

                    float b = kf->rgb_.at<uchar>(y, 3*x) / 255.0;
                    float g = kf->rgb_.at<uchar>(y, 3*x+1) / 255.0;
                    float r = kf->rgb_.at<uchar>(y, 3*x+2) / 255.0;
                    vr = r; vg = g; vb = b;

                    fileOut << "v " + std::to_string(Pw[0]) + " " + std::to_string(Pw[1]) + " " + std::to_string(Pw[2]) + " "
                               + std::to_string(vr) + " " << std::to_string(vg) + " " + std::to_string(vb) << std::endl;
                }
            }
        }
        kf->SetEraseSemiDense();

    }

    fileOut.flush();
    fileOut.close();
    std::cout << "saved semi dense point cloud" << std::endl;
}


ProbabilityMapping::ProbabilityMapping(ORB_SLAM2::Map* pMap)
{
    mpMap = pMap;
    mbFinishRequested = false; //init
    mbFinished = false;
    mpModeler = new Modeler(mpMap);
    mpMap->SetModeler(mpModeler);
}

void ProbabilityMapping::Run()
{
#ifdef OnlineLoop
    std::cout << "Online semi-dense mapping and line segment extraction" << std::endl;
#else
    std::cout << "Offline semi-dense mapping and line segment extraction" << std::endl;
#endif

    struct timespec start, finish;
    double duration;

    while(1)
    {
        if(CheckFinish()) break;
        {
#ifdef ForceRealTime
            unique_lock<mutex> lock(mMutexSemiDense);
#endif

#ifdef OnlineLoop
            SemiDenseLoop();
            //make point position dependent to kf position
            UpdateAllSemiDensePointSet();

            if (mpModeler->CheckNewTranscriptEntry()) {

                mpModeler->RunRemainder();

                mpModeler->UpdateModel();
            }
#endif
        }

		//handle reset
		ResetIfRequested();

        usleep(5000);
    }

    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();


    clock_gettime(CLOCK_MONOTONIC, &start);

    SemiDenseLoop();

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    duration *= 1000;
    std::cout << "semi dense mapping took total: "<< duration << "ms  avg:" << duration/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;

    SaveSemiDensePoints();

    // 
    mLineDetector.RunLine3Dpp(vpKFs);
    // 
    mLineDetector.LineFittingEDLinesOffline(vpKFs);

#ifndef OnlineLoop

    mLineDetector.LineFittingOffline(vpKFs, mpModeler);

    // add keyframe entry to carv
    for (int i = 0; i < (int)vpKFs.size(); i++){
        mpModeler->AddLineSegmentKeyFrameEntry(vpKFs[i]);
    }
#endif

    mLineDetector.SaveAllLineSegments();
    mLineDetector.SaveClusteredSegments();


    clock_gettime(CLOCK_MONOTONIC, &start);

    if (mpModeler->CheckNewTranscriptEntry()) {

        mpModeler->RunOnce();

        mpModeler->UpdateModel();
    }

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    duration *= 1000;
    std::cout << "modeling took total: "<< duration << "ms  avg:" << duration/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;
    mLineDetector.time_modeling.push_back(duration);
    mLineDetector.time_modeling.push_back(vpKFs.size());

    // save model.
    // WriteModel();

    mLineDetector.Summary();

    mbFinished = true;
}

/*
 *    TestSemiDenseViewer:
 *     add const depth to every pixel,  used to test show semidense in pangolin
 */
void ProbabilityMapping::TestSemiDenseViewer()
{
    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if(vpKFs.size() < 2)
    {
        return;
    }
    for(size_t i =0;i < vpKFs.size(); i++ )
    {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad() || pKF->semidense_flag_)
            continue;

        cv::Mat image = pKF->GetImage();
        std::vector<std::vector<depthHo> > temp_ho (image.rows, std::vector<depthHo>(image.cols, depthHo()) );

        for(int y = 0; y < image.rows; ){
            for(int x = 0; x < image.cols; ){

                depthHo dh;
                dh.depth = 100.0;   // const
                float X = dh.depth*(x- pKF->cx ) / pKF->fx;
                float Y = dh.depth*(y- pKF->cy ) / pKF->fy;
                cv::Mat Pc = (cv::Mat_<float>(4,1) << X, Y , dh.depth, 1); // point in camera frame.
                cv::Mat Twc = pKF->GetPoseInverse();
                cv::Mat pos = Twc * Pc;
                dh.Pw<< pos.at<float>(0),pos.at<float>(1),pos.at<float>(2);
                dh.supported = true;
                temp_ho[y][x] = dh;  // save point to keyframe semidense map

                x = x+4; // don't use all pixel to test
            }
            y = y+4;
        }
        pKF->semidense_flag_ = true;
    }
    cout<<"semidense_Info:    vpKFs.size()--> "<<vpKFs.size()<<std::endl;


}


void ProbabilityMapping::SemiDenseLoop(){

    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if(vpKFs.size() < 10){return;}

    for(size_t i =0;i < vpKFs.size(); i++ )
    {
        ORB_SLAM2::KeyFrame* kf = vpKFs[i];

        kf->SetNotEraseSemiDense();

        if(kf->isBad() || kf->semidense_flag_ || !kf->MappingIdDelay()) {
            kf->SetEraseSemiDense();
            continue;
        }

        // use covisN good neighbor kfs
        std::vector<ORB_SLAM2::KeyFrame*> closestMatchesAll = kf->GetVectorCovisibleKeyFrames();
        std::vector<ORB_SLAM2::KeyFrame*> closestMatches;
        for (size_t idxCov = 0; idxCov < closestMatchesAll.size(); idxCov++){
            if (closestMatches.size() >= covisN)
                break;
            ORB_SLAM2::KeyFrame* kfCM = closestMatchesAll[idxCov];
            kfCM->SetNotEraseSemiDense();
            if (kfCM->isBad() || !kfCM->Mapped()) {
                kfCM->SetEraseSemiDense();
                continue;
            }
            closestMatches.push_back(kfCM);
        }
        if(closestMatches.size() < covisN) {
            for (size_t idxCov = 0; idxCov < closestMatches.size(); idxCov++){
                closestMatches[idxCov]->SetEraseSemiDense();
            }
            kf->SetEraseSemiDense();
            continue;
        }

        cout<<"semidense_Info:    vpKFs.size()--> "<< vpKFs.size() << std::endl;

        // start timing
        struct timespec start, finish;
        double duration;
        clock_gettime(CLOCK_MONOTONIC, &start);

        // detect edge map
        mLineDetector.DetectEdgeMap(kf);

        // detect line segments
        mLineDetector.DetectLineSegments(kf);

        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"line segment detection took: "<< duration << "s" << std::endl;
        clock_gettime(CLOCK_MONOTONIC, &start);


        std::map<ORB_SLAM2::KeyFrame*,float> rotIs;
        for(size_t j = 0; j < closestMatches.size(); j++){
            std::vector<float> rot = GetRotInPlane(kf, closestMatches[j]);
            std::sort(rot.begin(), rot.end());
            // 0 rotation for kf pair without covisibility
            float medianRot = 0;
            if (rot.size() > 0)
                medianRot = rot[(rot.size()-1)/2];
            rotIs.insert(std::map<ORB_SLAM2::KeyFrame*,float>::value_type(closestMatches[j],medianRot));
        }

        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"compute inplane rotation took: "<< duration << "s" << std::endl;
        clock_gettime(CLOCK_MONOTONIC, &start);


        float max_depth;
        float min_depth;
        // get max_depth and min_depth in current key frame to limit search range
        StereoSearchConstraints(kf, &min_depth, &max_depth);

        cv::Mat image = kf->GetImage();

        std::vector <cv::Mat> F;
        F.clear();
        for(size_t j=0; j<closestMatches.size(); j++)
        {
            ORB_SLAM2::KeyFrame* kf2 = closestMatches[ j ];
            cv::Mat F12 = ComputeFundamental(kf,kf2);
            F.push_back(F12);
        }

        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"compute fundamental matrix took: "<< duration << "s" << std::endl;
        clock_gettime(CLOCK_MONOTONIC, &start);


#pragma omp parallel for schedule(dynamic) collapse(2)
        for(int y = kf->mnMinY; y < kf->mnMaxY; y++)
        {
            for(int x = kf->mnMinX; x< kf->mnMaxX; x++)
            {

                // speed up by only computing depth for edge pixels (optional)
                if(kf->mEdgeIndex.at<int>(y,x) < 0) continue;

                if(kf->GradImg.at<float>(y,x) <= lambdaG){continue;}
                float pixel = (float)image.at<uchar>(y,x); //maybe it should be cv::Mat

                std::vector<depthHo> depth_ho;
                depth_ho.clear();
                for(size_t j=0; j<closestMatches.size(); j++)
                {
                    ORB_SLAM2::KeyFrame* kf2 = closestMatches[ j ];
                    cv::Mat F12 = F[j];

                    float rot = kf->GradTheta.at<float>(y,x);
                    float rot2 = rotIs[kf2];
                    float best_u(0.0),best_v(0.0);
                    depthHo dh;
                    EpipolarSearch(kf, kf2, x, y, pixel, min_depth, max_depth, &dh,F12,best_u,best_v,rot,rot2);

                    if (dh.supported && 1/dh.depth > 0.0)
                    {
                        depth_ho.push_back(dh);
                    }
                }

                if (depth_ho.size() > lambdaN) {
                    depthHo dh_temp;
                    InverseDepthHypothesisFusion(depth_ho, dh_temp);
                    if(dh_temp.supported)
                    {
                        kf->depth_map_.at<float>(y,x) = dh_temp.depth;   //  used to do IntraKeyFrameDepthChecking
                        kf->depth_sigma_.at<float>(y,x) = dh_temp.sigma;
                    }
                }

            }
        }

        // Intra keyframe depth checking and growing are disabled for speed (optional)
//        std::cout<<"IntraKeyFrameDepthChecking " << i <<std::endl;
//        IntraKeyFrameDepthChecking( kf->depth_map_,  kf->depth_sigma_, kf->GradImg);
//        IntraKeyFrameDepthGrowing( kf->depth_map_,  kf->depth_sigma_, kf->GradImg);


        kf->semidense_flag_ = true;

        for (size_t idxCov = 0; idxCov < closestMatches.size(); idxCov++){
            closestMatches[idxCov]->SetEraseSemiDense();
        }
        kf->SetEraseSemiDense();

        // timing
        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"depth reconstruction took: "<< duration << "s" << std::endl;

    }

    for(size_t i =0;i < vpKFs.size(); i++ )
    {
        ORB_SLAM2::KeyFrame* kf = vpKFs[i];

        kf->SetNotEraseSemiDense();

        if(kf->isBad() || kf->interKF_depth_flag_ || !kf->MappingIdDelay() || !kf->semidense_flag_) {
            kf->SetEraseSemiDense();
            continue;
        }

        std::vector<ORB_SLAM2::KeyFrame*> closestMatchesAll = kf->GetVectorCovisibleKeyFrames();
        std::vector<ORB_SLAM2::KeyFrame*> closestMatches;
        for (size_t idxCov = 0; idxCov < closestMatchesAll.size(); idxCov++){
            if (closestMatches.size() >= covisN)
                break;
            ORB_SLAM2::KeyFrame* kfCM = closestMatchesAll[idxCov];
            kfCM->SetNotEraseSemiDense();
            if (kfCM->isBad() || !kfCM->Mapped() || !kfCM->semidense_flag_) {
                kfCM->SetEraseSemiDense();
                continue;
            }
            closestMatches.push_back(kfCM);
        }
        if(closestMatches.size() < covisN) {
            for (size_t idxCov = 0; idxCov < closestMatches.size(); idxCov++){
                closestMatches[idxCov]->SetEraseSemiDense();
            }
            kf->SetEraseSemiDense();
            continue;
        }

        // start timing
        struct timespec start, finish;
        double duration;
        clock_gettime(CLOCK_MONOTONIC, &start);

        std::cout << "InterKeyFrameDepthChecking " << i << std::endl;
        InterKeyFrameDepthChecking(kf,closestMatches);

        UpdateSemiDensePointSet(kf);

        kf->interKF_depth_flag_ = true;

        // unlock neighbors
        for (size_t idxCov = 0; idxCov < closestMatches.size(); idxCov++){
            closestMatches[idxCov]->SetEraseSemiDense();
        }

        //timing
        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"depth checking took: "<< duration << "s" << std::endl;


#ifdef OnlineLoop

        clock_gettime(CLOCK_MONOTONIC, &start);

        mLineDetector.LineFitting(kf);

        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"line fitting took: "<< duration << "s" << std::endl;

        clock_gettime(CLOCK_MONOTONIC, &start);

        mLineDetector.MergeLines(kf, mpModeler);

        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"line merging took: "<< duration << "s" << std::endl;

        // carv: add keyframe after line segment extraction
        mpModeler->AddLineSegmentKeyFrameEntry(kf);

#endif

        // unlock the keyframe
        kf->SetEraseSemiDense();

    }
}


void ProbabilityMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }
    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void ProbabilityMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        Modeler* newModeler = new Modeler(mpMap);
        mpMap->SetModeler(newModeler);
        newModeler->UpdateModel();
        delete mpModeler;
        mpModeler = newModeler;
        mLineDetector.Reset();
        mbResetRequested=false;
    }
}

void ProbabilityMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ProbabilityMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ProbabilityMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool ProbabilityMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


void ProbabilityMapping::ApplySigmaThreshold(ORB_SLAM2::KeyFrame* kf){

#pragma omp parallel for schedule(dynamic) collapse(2)
    for(int y = 0+2; y < kf->im_.rows-2; y++)
    {
        for(int x = 0+2; x< kf->im_.cols-2; x++)
        {

            if (kf->depth_map_.at<float>(y,x) < 0.000001) {
                continue;
            }

            if (kf->depth_sigma_.at<float>(y,x) > 0.025) {
                kf->depth_map_.at<float>(y,x) = 0.0;
            }

        }
    }

}

void ProbabilityMapping::UpdateAllSemiDensePointSet(){

    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if(vpKFs.size() < 10){return;}

    for (size_t i = 0; i < vpKFs.size(); i++){
        ORB_SLAM2::KeyFrame* kf = vpKFs[i];
        kf->SetNotEraseSemiDense();
        if( kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_ || !kf->MappingIdDelay()) {
            kf->SetEraseSemiDense();
            continue;
        }

        if (kf->PoseChanged()) {
            UpdateSemiDensePointSet(kf);
            kf->SetPoseChanged(false);
        }
        kf->SetEraseSemiDense();
    }
}


void ProbabilityMapping::UpdateSemiDensePointSet(ORB_SLAM2::KeyFrame* kf){
    unique_lock<mutex> lock(kf->mMutexSemiDensePoints);

#pragma omp parallel for schedule(dynamic) collapse(2)
    for(int y = 0+2; y < kf->im_.rows-2; y++)
    {
        for(int x = 0+2; x< kf->im_.cols-2; x++)
        {

            if (kf->depth_map_checked_.at<float>(y,x) < 0.000001) {
                kf->SemiDensePointSets_.at<float>(y,3*x+0) = 0.0;
                kf->SemiDensePointSets_.at<float>(y,3*x+1) = 0.0;
                kf->SemiDensePointSets_.at<float>(y,3*x+2) = 0.0;
                continue;
            }

            float inv_d = kf->depth_map_checked_.at<float>(y,x);
            float Z = 1/inv_d ;
            float X = Z *(x- kf->cx ) / kf->fx;
            float Y = Z *(y- kf->cy ) / kf->fy;

            cv::Mat Pc = (cv::Mat_<float>(4,1) << X, Y , Z, 1); // point in camera frame.
            cv::Mat Twc = kf->GetPoseInverse();
            cv::Mat pos = Twc * Pc;

            kf->SemiDensePointSets_.at<float>(y,3*x+0) = pos.at<float>(0);
            kf->SemiDensePointSets_.at<float>(y,3*x+1) = pos.at<float>(1);
            kf->SemiDensePointSets_.at<float>(y,3*x+2) = pos.at<float>(2);
        }
    }

}


void ProbabilityMapping::StereoSearchConstraints(ORB_SLAM2::KeyFrame* kf, float* min_depth, float* max_depth){
    std::vector<float> orb_depths = kf->GetAllPointDepths();

    float sum = std::accumulate(orb_depths.begin(), orb_depths.end(), 0.0);
    float mean = sum / orb_depths.size();

    std::vector<float> diff(orb_depths.size());
    std::transform(orb_depths.begin(), orb_depths.end(), diff.begin(), std::bind2nd(std::minus<float>(), mean));
    float variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)/orb_depths.size();
    float stdev = std::sqrt(variance);

    *max_depth = 1/(mean + 2 * stdev);
    *min_depth = 1/(mean - 2 * stdev);
}

void ProbabilityMapping::EpipolarSearch(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame *kf2, const int x, const int y, float pixel,
                                        float min_depth, float max_depth, depthHo *dh,cv::Mat F12,float& best_u,float& best_v,float th_pi,float rot)
{

    float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
    float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
    float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);

    if((a/b)< -4 || a/b> 4) return;   // if epipolar direction is approximate to perpendicular, we discard it.  May be product wrong match.

    float old_err = 100000.0;
    float best_photometric_err = 0.0;
    float best_gradient_modulo_err = 0.0;
    int best_pixel = 0;

    int uj_plus,uj_minus;
    float vj,vj_plus,vj_minus;
    float g, q,denomiator ,ustar , ustar_var;

    float umin(0.0),umax(0.0);
    GetSearchRange(umin,umax,x,y,min_depth,max_depth,kf1,kf2);
    for(int uj = (int)std::ceil(umin); uj <= std::floor(umax); uj++)
    {
        vj = -((a/b)*uj+(c/b));
        if(std::floor(vj) < 0 || std::ceil(vj) >= kf2->im_.rows ){continue;}

        uj_plus = uj + 1;
        uj_minus = uj - 1;

        if(uj_plus >= kf2->mnMaxX) {continue;}
        if(uj_minus < kf2->mnMinX) {continue;}

        vj_plus = -((a/b)*uj_plus + (c/b));
        vj_minus = -((a/b)*uj_minus + (c/b));

        if(std::floor(vj_plus) < kf2->mnMinY || std::ceil(vj_plus) >= kf2->mnMaxY ){continue;}
        if(std::floor(vj_minus) < kf2->mnMinY || std::ceil(vj_minus) >= kf2->mnMaxY ){continue;}

        // condition 1:
        if( ylinear<float>(kf2->GradImg,vj,uj) <= lambdaG){continue;}

        // condition 2:
        float th_epipolar_line = cv::fastAtan2(-a/b,1);
        float temp_gradth =  yangle<float>(kf2->GradTheta,vj,uj);
        float ang_diff = temp_gradth - th_epipolar_line;
        if(ang_diff >= 360) { ang_diff -= 360; }
        if(ang_diff < 0) { ang_diff += 360; }
        if(ang_diff > 180) ang_diff = 360 - ang_diff;
        if(ang_diff > 90) ang_diff = 180 - ang_diff;
        if(ang_diff >= lambdaL) { continue; }

        // condition 3:
        float ang_pi_rot = th_pi + rot;
        if(ang_pi_rot >= 360) { ang_pi_rot -= 360; }
        if(ang_pi_rot < 0) { ang_pi_rot += 360; }
        float th_diff = temp_gradth - ang_pi_rot;
        if(th_diff >= 360) { th_diff -= 360; }
        if(th_diff < 0) { th_diff += 360; }
        if(th_diff > 180) th_diff = 360 - th_diff;
        if(th_diff >= lambdaTheta) continue;

        float photometric_err = pixel - ylinear<uchar>(kf2->im_,vj,uj);
        float gradient_modulo_err = kf1->GradImg.at<float>(y,x)  - ylinear<float>( kf2->GradImg,vj,uj);

        float err = (photometric_err*photometric_err  + (gradient_modulo_err*gradient_modulo_err)/(float)THETA);
        if(err < old_err)
        {
            best_pixel = uj;
            old_err = err;
            best_photometric_err = photometric_err;
            best_gradient_modulo_err = gradient_modulo_err;
        }
    }

    if(old_err < 100000.0)
    {

        uj_plus = best_pixel + 1;
        uj_minus = best_pixel - 1;

        vj_plus = -((a/b)*uj_plus + (c/b));
        vj_minus = -((a/b)*uj_minus + (c/b));

        g = (ylinear<uchar>(kf2->im_,vj_plus,uj_plus) - ylinear<uchar>(kf2->im_,vj_minus,uj_minus)) / 2;
        q = (ylinear<float>(kf2->GradImg,vj_plus,uj_plus) -  ylinear<float>(kf2->GradImg,vj_minus,uj_minus)) / 2;

        denomiator = (g*g + (1/(float)THETA)*q*q);
        ustar = best_pixel + (g*best_photometric_err + (1/(float)THETA)*q*best_gradient_modulo_err)/denomiator;
        ustar_var = (2*kf2->I_stddev*kf2->I_stddev/denomiator);

        best_u = ustar;
        best_v =  -( (a/b)*best_u + (c/b) );

        ComputeInvDepthHypothesis(kf1, kf2, ustar, ustar_var, a, b, c, dh,x,y);
    }

}

std::vector<float> ProbabilityMapping::GetRotInPlane(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2){
    std::vector<ORB_SLAM2::MapPoint*> vMPs1 = kf1->GetMapPointMatches();
    std::vector<ORB_SLAM2::MapPoint*> vMPs2 = kf2->GetMapPointMatches();
    std::vector<float> rotInPlane;
    for (size_t idx1 = 0; idx1 < vMPs1.size(); idx1++){
        if (vMPs1[idx1]){
            for (size_t idx2 = 0; idx2 < vMPs2.size(); idx2++) {
                if (vMPs2[idx2] == vMPs1[idx1]) {
                    float angle1 = kf1->GetKeyPointsUn()[idx1].angle;
                    float angle2 = kf2->GetKeyPointsUn()[idx2].angle;
                    if (angle1 < 0 || angle2 < 0) continue;
                    rotInPlane.push_back(angle2 - angle1);
                }
            }
        }
    }
    return rotInPlane;
}

void ProbabilityMapping::IntraKeyFrameDepthChecking(cv::Mat& depth_map, cv::Mat& depth_sigma,const cv::Mat gradimg)
{
    cv::Mat depth_map_new = depth_map.clone();
    cv::Mat depth_sigma_new = depth_sigma.clone();

#pragma omp parallel for schedule(dynamic) collapse(2)
    for (int py = 2; py < (depth_map.rows - 2); py++)
    {
        for (int px = 2; px < (depth_map.cols - 2); px++)
        {

            if (depth_map.at<float>(py,px) > 0.000001)  // if  d !=0.0
            {
                std::vector<depthHo> compatible_neighbor_ho;

                depthHo dha,dhb;
                dha.depth = depth_map.at<float>(py,px);
                dha.sigma = depth_sigma.at<float>(py,px);
                for (int y = py - 1; y <= py + 1; y++)
                {
                    for (int x = px - 1; x <= px + 1; x++)
                    {

                        if (x == px && y == py) continue;
                        if( depth_map.at<float>(y,x) > 0.000001)
                        {
                            if(ChiTest(depth_map.at<float>(y,x),depth_map.at<float>(py,px),depth_sigma.at<float>(y,x),depth_sigma.at<float>(py,px)))
                            {
                                dhb.depth = depth_map.at<float>(y,x);
                                dhb.sigma = depth_sigma.at<float>(y,x);
                                compatible_neighbor_ho.push_back(dhb);
                            }

                        }
                    }
                }
                compatible_neighbor_ho.push_back(dha);  // dont forget itself.

                if (compatible_neighbor_ho.size() >= 3)
                {
                    depthHo fusion;
                    float min_sigma = 0;
                    GetFusion(compatible_neighbor_ho, fusion, &min_sigma);

                    depth_map_new.at<float>(py,px) = fusion.depth;
                    depth_sigma_new.at<float>(py,px) = min_sigma;

                } else
                {
                    depth_map_new.at<float>(py,px) = 0.0;
                    depth_sigma_new.at<float>(py,px) = 0.0;

                }

            }
        }
    }

    depth_map = depth_map_new.clone();
    depth_sigma = depth_sigma_new.clone();

}

void ProbabilityMapping::IntraKeyFrameDepthGrowing(cv::Mat& depth_map, cv::Mat& depth_sigma,const cv::Mat gradimg)
{
    cv::Mat depth_map_new = depth_map.clone();
    cv::Mat depth_sigma_new = depth_sigma.clone();

#pragma omp parallel for schedule(dynamic) collapse(2)
    for (int py = 2; py < (depth_map.rows - 2); py++)
    {
        for (int px = 2; px < (depth_map.cols - 2); px++)
        {

            if (depth_map.at<float>(py,px) < 0.000001)  // if  d ==0.0 : grow the reconstruction getting more density
            {
                if(gradimg.at<float>(py,px)<=lambdaG) continue;
                //search supported  by at least 2 of its 8 neighbours pixels
                std::vector< std::pair<float,float> > supported;

                for( int  y = py - 1 ; y <= py+1; y++)
                    for( int  x = px - 1 ; x <= px+1; x++)
                    {
                        if(x == px && y == py) continue;

                        if(ChiTest(depth_map.at<float>(y,x),depth_map.at<float>(py,px),depth_sigma.at<float>(y,x),depth_sigma.at<float>(py,px)))
                        {
                            std::pair<float, float> depth;
                            depth.first = depth_map.at<float>(y,x);
                            depth.second = depth_sigma.at<float>(y,x);
                            supported.push_back(depth);
                        }

                    }

                if(supported.size() >= 2)
                {
                    float d(0.0),s(0.0);
                    GetFusion(supported,d,s);
                    depth_map_new.at<float>(py,px) = d;
                    depth_sigma_new.at<float>(py,px) = s;
                }
            }

        }
    }

    depth_map = depth_map_new.clone();
    depth_sigma = depth_sigma_new.clone();

}

void ProbabilityMapping::InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo& dist) {
    dist.depth = 0;
    dist.sigma = 0;
    dist.supported = false;

    std::vector<depthHo> compatible_ho;
    std::vector<depthHo> compatible_ho_temp;
    float chi = 0;

    for (size_t a=0; a < h.size(); a++) {

        compatible_ho_temp.clear();
        for (size_t b=0; b < h.size(); b++)
        {
            if (a==b) {
                compatible_ho_temp.push_back(h[b]);
                continue;
            }
            if (ChiTest(h[a], h[b], &chi))
            {compatible_ho_temp.push_back(h[b]);}// test if the hypotheses a and b are compatible
        }

        if (compatible_ho.size() < compatible_ho_temp.size()){
            compatible_ho.swap(compatible_ho_temp);
        }
    }

    // calculate the parameters of the inverse depth distribution by fusing hypotheses
    if (compatible_ho.size() > lambdaN) {
        GetFusion(compatible_ho, dist, &chi);
    }
}

void ProbabilityMapping::InterKeyFrameDepthChecking(const cv::Mat& im, ORB_SLAM2::KeyFrame* currentKf, std::vector<std::vector<depthHo> >& h) {
    std::vector<ORB_SLAM2::KeyFrame*> neighbors;

    neighbors = currentKf->GetBestCovisibilityKeyFrames(covisN);

    // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
    // and propagate inverse depth
    for (int px = 0; px < im.rows; px++) {
        for (int py = 0; py < im.cols; py++) {
            if (h[px][py].supported == false) continue;

            float depthp = h[px][py].depth;
            // count of neighboring keyframes in which there is at least one compatible pixel
            int compatible_neighbor_keyframes_count = 0;
            // keep track of compatible pixels for the gauss-newton step
            std::vector<depthHo> compatible_pixels_by_frame[neighbors.size()];
            int n_compatible_pixels = 0;

            for(size_t j=0; j<neighbors.size(); j++) {
                ORB_SLAM2::KeyFrame* pKFj = neighbors[j];

                cv::Mat kj = pKFj->GetCalibrationMatrix();
                cv::Mat xp;
                GetXp(kj, px, py, &xp);

                cv::Mat rcwj = pKFj->GetRotation();
                cv::Mat tcwj = pKFj->GetTranslation();

                // Eq (12)
                // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                cv::Mat temp;
                float denom1, denom2;

                temp = rcwj.row(2) * xp;
                denom1 = temp.at<float>(0,0);
                temp = depthp * tcwj.at<float>(2);
                denom2 = temp.at<float>(0,0);
                float depthj = depthp / (denom1 + denom2);

                cv::Mat xj2d = (kj * rcwj * (1 / depthp) * xp) + (kj * tcwj);
                float xj = xj2d.at<float>(0,0);
                float yj = xj2d.at<float>(1,0);

                std::vector<depthHo> compatible_pixels;
                // look in 4-neighborhood pixel p_j,n around xj for compatible inverse depth
                int pxn = floor(xj);
                int pyn = floor(yj);
                for (int nx = pxn-1; nx <= pxn + 1; nx++) {
                    for (int ny = pyn-1; ny < pyn + 1; ny++) {
                        if ((nx == ny) || ((nx - pxn) && (ny - pyn))) continue;
                        if (!h[nx][ny].supported) continue;
                        // Eq (13)
                        float depthjn = h[nx][ny].depth;
                        float sigmajn = h[nx][ny].sigma;
                        float test = pow((depthj - depthjn), 2) / pow(sigmajn, 2);
                        if (test < 3.84) {
                            compatible_pixels.push_back(h[nx][ny]);
                        }
                    }
                }
                compatible_pixels_by_frame[j] = compatible_pixels; // is this a memory leak?
                n_compatible_pixels += compatible_pixels.size();

                // at least one compatible pixel p_j,n must be found in at least lambdaN neighbor keyframes
                if (compatible_pixels.size()) {
                    compatible_neighbor_keyframes_count++;
                }
            } // for j = 0...neighbors.size()-1

            // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
            if (compatible_neighbor_keyframes_count < lambdaN) {
                h[px][py].supported = false;
            } else {
                // gauss-newton step to minimize depth difference in all compatible pixels
                // need 1 iteration since depth propagation eq. is linear in depth
                float argmin = depthp;
                cv::Mat J(n_compatible_pixels, 1, CV_32F);
                cv::Mat R(n_compatible_pixels, 1, CV_32F);
                int n_compat_index = 0;
                int iter = 1;
                for (int k = 0; k < iter; k++) {
                    for (size_t j = 0; j < neighbors.size(); j++) {
                        cv::Mat xp;
                        GetXp(neighbors[j]->GetCalibrationMatrix(), px, py, &xp);

                        cv::Mat rji = neighbors[j]->GetRotation();
                        cv::Mat tji = neighbors[j]->GetTranslation();
                        for (size_t i = 0; i < compatible_pixels_by_frame[j].size(); i++) {
                            float ri = 0;
                            Equation14(compatible_pixels_by_frame[j][i], argmin, xp, rji, tji, &ri);
                            R.at<float>(n_compat_index, 0) = ri;

                            cv::Mat tempm = rji.row(2) * xp;
                            float tempf = tempm.at<float>(0,0);
                            depthHo tempdH = compatible_pixels_by_frame[j][i];
                            J.at<float>(n_compat_index, 0) = -1 * tempf / (pow(tempdH.depth, 2) * tempdH.sigma);

                            n_compat_index++;
                        }
                    }
                    cv::Mat temp = J.inv(cv::DECOMP_SVD) * R;
                    argmin = argmin - temp.at<float>(0,0);
                }
                h[px][py].depth = argmin;
            }
        } // for py = 0...im.cols-1
    } // for px = 0...im.rows-1
}


void ProbabilityMapping::InterKeyFrameDepthChecking(ORB_SLAM2::KeyFrame* currentKf, std::vector<ORB_SLAM2::KeyFrame*> neighbors) {

    // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
    // and propagate inverse depth

    std::vector <cv::Mat> Rji,tji;
    for(size_t j=0; j<neighbors.size(); j++)
    {
        ORB_SLAM2::KeyFrame* kf2 = neighbors[ j ];

        cv::Mat Rcw1 = currentKf->GetRotation();
        cv::Mat tcw1 = currentKf->GetTranslation();
        cv::Mat Rcw2 = kf2->GetRotation();
        cv::Mat tcw2 = kf2->GetTranslation();

        cv::Mat R21 = Rcw2*Rcw1.t();
        cv::Mat t21 = -Rcw2*Rcw1.t()*tcw1+tcw2;

        Rji.push_back(R21);
        tji.push_back(t21);

    }

    int cols = currentKf->im_.cols;
    int rows = currentKf->im_.rows;
    float fx = currentKf->fx;
    float fy = currentKf->fy;
    float cx = currentKf->cx;
    float cy = currentKf->cy;

#pragma omp parallel for schedule(dynamic) collapse(2)
    for (int py = 2; py <rows-2; py++) {
        for (int px = 2; px < cols-2; px++) {

            //  if d == 0.0  continue;
            if (currentKf->depth_map_.at<float>(py,px) < 0.000001) {
                currentKf->depth_map_checked_.at<float>(py,px) = 0.0;
                continue;
            }

            float depthp = currentKf->depth_map_.at<float>(py,px);
            // count of neighboring keyframes in which there is at least one compatible pixel
            int compatible_neighbor_keyframes_count = 0;

            // keep track of compatible pixels for the gauss-newton step
            std::vector<std::vector<depthHo>> compatible_pixels_by_frame;
            int num_compatible_pixels = 0;

            for(size_t j=0; j<neighbors.size(); j++) {

                ORB_SLAM2::KeyFrame* pKFj = neighbors[j];
                cv::Mat K = pKFj->GetCalibrationMatrix();

                cv::Mat xp=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);// inverse project.    if has distortion, this code shoud fix
                cv::Mat temp = Rji[j] * xp /depthp + tji[j];
                cv::Mat Xj = K*temp;
                Xj = Xj/Xj.at<float>(2);   //   u = u'/z   ,  v = v'/z

                // Eq (12)
                // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                temp = Rji[j].row(2) * xp;
                float denom1 = temp.at<float>(0,0);
                temp = depthp * tji[j].at<float>(2);
                float denom2 = temp.at<float>(0,0);
                float depthj = depthp / (denom1 + denom2);

                float xj = Xj.at<float>(0);
                float yj = Xj.at<float>(1);

                // look in 4-neighborhood pixel p_j,n around xj for compatible inverse depth
                std::vector<depthHo> compatible_pixels_J;
                if (xj<0 || xj>=cols-1 || yj<0 || yj>=rows-1) {
                    // make sure kf j correspondence
                    compatible_pixels_by_frame.push_back(compatible_pixels_J);
                    continue;
                }
                int x0 = (int)std::floor(xj);
                int y0 = (int )std::floor(yj);
                int x1 = x0 + 1;
                int y1 =  y0 + 1;

                float d = pKFj->depth_map_.at<float>(y0,x0);
                float sigma = pKFj->depth_sigma_.at<float>(y0,x0);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }
                d = pKFj->depth_map_.at<float>(y1,x0);
                sigma = pKFj->depth_sigma_.at<float>(y1,x0);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }
                d = pKFj->depth_map_.at<float>(y0,x1);
                sigma = pKFj->depth_sigma_.at<float>(y0,x1);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }
                d = pKFj->depth_map_.at<float>(y1,x1);
                sigma = pKFj->depth_sigma_.at<float>(y1,x1);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }

                // at least one compatible pixel p_j,n must be found in at least lambdaN neighbor keyframes
                if (compatible_pixels_J.size() >= 1) {compatible_neighbor_keyframes_count++;}
                compatible_pixels_by_frame.push_back(compatible_pixels_J);
                num_compatible_pixels += compatible_pixels_J.size();

            } // for j = 0...neighbors.size()-1

            // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
            if (compatible_neighbor_keyframes_count < lambdaN )
            {
                currentKf->depth_map_checked_.at<float>(py,px) = 0.0;
            }
            else {
                // gauss newton smoothing
                cv::Mat xp=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);
                float dp = 1/depthp;

                cv::Mat J = cv::Mat(num_compatible_pixels,1,CV_32F);
                cv::Mat r0 = cv::Mat(num_compatible_pixels,1,CV_32F);
                int idxJN = 0;
                for (size_t j = 0; j < compatible_pixels_by_frame.size(); j++){
                    std::vector<depthHo>& compatibleJ = compatible_pixels_by_frame[j];
                    for (size_t n = 0; n < compatibleJ.size(); n++){
                        cv::Mat rzxp = Rji[j].row(2) * xp;
                        float djn = 1/compatibleJ[n].depth;
                        float sigmajn = compatibleJ[n].sigma;
                        float d2sigma = djn*djn * sigmajn;

                        J.at<float>(idxJN,0) = - rzxp.at<float>(0,0) / d2sigma;
                        r0.at<float>(idxJN,0) = (djn - dp*rzxp.at<float>(0,0) - tji[j].at<float>(2,0)) / d2sigma;

                        idxJN++;
                    }
                }
                cv::Mat Jtr0 = - J.t() * r0;
                cv::Mat JtJ = J.t() * J;

                float dpDelta = Jtr0.at<float>(0,0) / JtJ.at<float>(0,0);

                currentKf->depth_map_checked_.at<float>(py,px) = 1/ (dp + dpDelta);
            }

        } // for py = 0...im.cols-1
    } // for px = 0...im.rows-1

}

void ProbabilityMapping::Equation14(depthHo& dHjn, float& depthp, cv::Mat& xp, cv::Mat& rji, cv::Mat& tji, float* res) {
    cv::Mat tempm = rji.row(2) * xp;
    float tempf = tempm.at<float>(0,0);
    float tji_z = tji.at<float>(2);
    *res = pow((dHjn.depth - (depthp * tempf) - tji_z) / (pow(dHjn.depth, 2) * dHjn.sigma), 1);
}


////////////////////////
// Utility functions
////////////////////////

void ProbabilityMapping::ComputeInvDepthHypothesis(ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame* kf2, float ustar, float ustar_var,
                                                   float a, float b, float c,ProbabilityMapping::depthHo *dh, int x,int y) {

    float inv_pixel_depth =  0.0;

    // equation 8 comput depth
    GetPixelDepth(ustar, x , y,kf, kf2,inv_pixel_depth);

    float ustar_min = ustar - sqrt(ustar_var);

    float inv_depth_min = 0.0;
    GetPixelDepth(ustar_min,x,y,kf,kf2, inv_depth_min);

    float ustar_max = ustar +  sqrt(ustar_var);

    float inv_depth_max = 0.0;
    GetPixelDepth(ustar_max,x,y,kf, kf2,inv_depth_max);

    // Equation 9
    float sigma_depth = cv::max(abs(inv_depth_max-inv_pixel_depth), abs(inv_depth_min-inv_pixel_depth));

    dh->depth = inv_pixel_depth;
    dh->sigma = sigma_depth;
    dh->supported = true;

}

void ProbabilityMapping::GetGradientMagAndOri(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* mag, cv::Mat* ori) {

    *gradx = cv::Mat::zeros(image.rows, image.cols, CV_32F);
    *grady = cv::Mat::zeros(image.rows, image.cols, CV_32F);
    *mag =  cv::Mat::zeros(image.rows, image.cols, CV_32F);
    *ori = cv::Mat::zeros(image.rows, image.cols, CV_32F);

    cv::Scharr(image, *gradx, CV_32F, 1, 0, 1/32.0);
    cv::Scharr(image, *grady, CV_32F, 0, 1, 1/32.0);

    cv::magnitude(*gradx,*grady,*mag);
    cv::phase(*gradx,*grady,*ori,true);

}

//might be a good idea to store these when they get calculated during ORB-SLAM.
void ProbabilityMapping::GetInPlaneRotation(ORB_SLAM2::KeyFrame* k1, ORB_SLAM2::KeyFrame* k2, float* th) {
    std::vector<cv::KeyPoint> vKPU1 = k1->GetKeyPointsUn();
    DBoW2::FeatureVector vFeatVec1 = k1->GetFeatureVector();
    std::vector<ORB_SLAM2::MapPoint*> vMapPoints1 = k1->GetMapPointMatches();
    cv::Mat Descriptors1 = k1->GetDescriptors();

    std::vector<cv::KeyPoint> vKPU2 = k2->GetKeyPointsUn();
    DBoW2::FeatureVector vFeatVec2 = k2->GetFeatureVector();
    std::vector<ORB_SLAM2::MapPoint*> vMapPoints2 = k2 ->GetMapPointMatches();
    cv::Mat Descriptors2 = k2->GetDescriptors();

    std::vector<int> rotHist[histo_length];
    for(int i=0;i<histo_length;i++)
        rotHist[i].reserve(500);//DescriptorDistance

    const float factor = 1.0f;//histo_length;

    DBoW2::FeatureVector::iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end) {
        if(f1it->first == f2it->first){
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++){
                size_t index1 = f1it->second[i1];

                ORB_SLAM2::MapPoint* pMP1 = vMapPoints1[index1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                cv::Mat d1 = Descriptors1.row(index1);

                int bestDist1 = INT_MAX;
                int bestDist2 = INT_MAX;
                size_t index2;
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++){
                    index2 = f2it->second[i2];

                    ORB_SLAM2::MapPoint* pMP2 = vMapPoints2[index2];
                    if(!pMP2)
                        continue;
                    if(pMP2->isBad())
                        continue;

                    cv::Mat d2 = Descriptors2.row(index2);

                    int dist = ORB_SLAM2::ORBmatcher::DescriptorDistance(d1,d2);

                    if(dist<bestDist1){
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                    }
                    else if(dist<bestDist2){
                        bestDist2 = dist;
                    }
                }
                if(bestDist1<th_low){
                    if(static_cast<float>(bestDist1)<NNRATIO*static_cast<float>(bestDist2)){
                        float rot = vKPU1[index1].angle - vKPU2[index2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==histo_length)
                            bin=0;
                        rotHist[bin].push_back(index1);
                    }
                }
            }
        }
    }
    //calculate the median angle
    size_t size = 0;
    for(int i=0;i<histo_length;i++)
        size += rotHist[i].size();

    size_t count = 0;
    for(int i=0;i<histo_length;i++) {
        for (size_t j=0; j < rotHist[i].size(); j++) {
            if (count==(size/2))
                *th = 360 * (float)(i) / histo_length;
            count++;
        }
    }

}


void ProbabilityMapping::PixelNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py, std::vector<depthHo>& support) {
    support.clear();
    float chi = 0;
    for (int y = py - 1; y <= py + 1; y++) {
        for (int x = px - 1; x <= px + 1; x++) {

            if (x == px && y == py) continue;
            if(!H[y][x].supported) continue;

            if (ChiTest(H[y][x], H[py][px], &chi))
            {
                support.push_back(H[y][x]);
            }
        }
    }
    support.push_back(H[py][px]);  // dont forget itself.
}

void ProbabilityMapping::PixelNeighborNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py, std::vector<std::vector<depthHo> >& support) {
    support.clear();
    float chi = 0;
    for (int x = px - 1; x <= px + 1; x++) {
        for (int y = py - 1; y <= py + 1; y++) {
            if (x == px && y == py) continue;
            std::vector<depthHo> tempSupport;
            for (int nx = px - 1; nx <= px + 1; nx++) {
                for (int ny = py - 1; ny <= py + 1; ny++) {
                    if ((nx == px && ny == py) || (nx == x && ny == y)) continue;
                    if (ChiTest(H[x][y], H[nx][ny], &chi)) {
                        tempSupport.push_back(H[nx][ny]);
                    }
                }
            }
            support.push_back(tempSupport);
        }
    }
}

void ProbabilityMapping::GetIntensityGradient_D(const cv::Mat& ImGrad, float a, float b, float c, int px, float* q) {
    int uplusone = px + 1;
    int vplusone =- ((a/b)*uplusone + (c/b));
    int uminone = px - 1;
    int vminone = -((a/b)*uminone + (c/b));
    *q = (ImGrad.at<float>(uplusone,vplusone) - ImGrad.at<float>(uminone,vminone))/2;
}

void ProbabilityMapping::GetTR(ORB_SLAM2::KeyFrame* kf, cv::Mat* t, cv::Mat* r) {

    cv::Mat Rcw2 = kf->GetRotation();
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = kf->GetTranslation();
    cv::Mat Tcw2(3,4,CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0,3));
    tcw2.copyTo(Tcw2.col(3));

    *t = Tcw2;
    *r = Rcw2;
}

void ProbabilityMapping::GetParameterization(const cv::Mat& F12, const int x, const int y, float& a, float& b, float& c) {
    // parameterization of the fundamental matrix (function of horizontal coordinate)
    // could probably use the opencv built in function instead
    a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
    b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
    c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
}

//Xp = K-1 * xp (below Equation 8)
// map 2D pixel coordinate to 3D point
void ProbabilityMapping::GetXp(const cv::Mat& k, int px, int py, cv::Mat* xp) {

    cv::Mat xp2d = cv::Mat(3,1,CV_32F);

    xp2d.at<float>(0,0) = px;
    xp2d.at<float>(1,0) = py;
    xp2d.at<float>(2,0) = 1;

    *xp = k.inv() * xp2d;
}

// Linear Triangulation Method
void ProbabilityMapping::GetPixelDepth(float uj, float vj, int px, int py, ORB_SLAM2::KeyFrame* kf,ORB_SLAM2::KeyFrame* kf2, float &p,ProbabilityMapping::depthHo *dh)
{

    float fx = kf->fx;
    float fy = kf->fy;
    float cx = kf->cx;
    float cy = kf->cy;

    cv::Mat R1w = kf->GetRotation();
    cv::Mat t1w = kf->GetTranslation();
    cv::Mat T1w(3,4,CV_32F);
    R1w.copyTo(T1w.colRange(0,3));  // 0,1,2 cols
    t1w.copyTo(T1w.col(3));

    cv::Mat R2w = kf2->GetRotation();
    cv::Mat t2w = kf2->GetTranslation();
    cv::Mat T2w(3,4,CV_32F);
    R2w.copyTo(T2w.colRange(0,3));
    t2w.copyTo(T2w.col(3));

    // inverse project.    if has distortion, this code shoud fix
    cv::Mat xn1 = (cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);
    cv::Mat xn2 = (cv::Mat_<float>(3,1) << (uj-cx)/fx, (vj-cy)/fy, 1.0);

    cv::Mat A(4,4,CV_32F);
    A.row(0) = xn1.at<float>(0) * T1w.row(2) - T1w.row(0);
    A.row(1) = xn1.at<float>(1) * T1w.row(2) - T1w.row(1);
    A.row(2) = xn2.at<float>(0) * T2w.row(2) - T2w.row(0);
    A.row(3) = xn2.at<float>(1) * T2w.row(2) - T2w.row(1);

    cv::Mat w,u,vt;
    cv::SVD::compute(A,w,u,vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat pw = vt.row(3).t();
    if(pw.at<float>(3) == 0) return;

    cv::Mat pw_normalize = pw.rowRange(0,3) / pw.at<float>(3) ; // Point at world frame.

    cv::Mat x3Dt = pw_normalize.t();
    float z1 = R1w.row(2).dot(x3Dt)+t1w.at<float>(2);
    p = 1/z1;

}

// Equation (8)
void ProbabilityMapping::GetPixelDepth(float uj, int px, int py, ORB_SLAM2::KeyFrame* kf,ORB_SLAM2::KeyFrame* kf2, float &p) {

    float fx = kf->fx;
    float cx = kf->cx;
    float fy = kf->fy;
    float cy = kf->cy;

    float ucx = uj - cx;

    cv::Mat Rcw1 = kf->GetRotation();
    cv::Mat tcw1 = kf->GetTranslation();
    cv::Mat Rcw2 = kf2->GetRotation();
    cv::Mat tcw2 = kf2->GetTranslation();

    cv::Mat R21 = Rcw2*Rcw1.t();
    cv::Mat t21 = -Rcw2*Rcw1.t()*tcw1+tcw2;

    cv::Mat xp=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);// inverse project.    if has distortion, this code shoud fix

    cv::Mat temp = R21.row(2) * xp * ucx;
    float num1 = temp.at<float>(0,0);
    temp = fx * (R21.row(0) * xp);
    float num2 = temp.at<float>(0,0);
    float denom1 = -t21.at<float>(2) * ucx;
    float denom2 = fx * t21.at<float>(0);

    p = (num1 - num2) / (denom1 + denom2);

}

void ProbabilityMapping::GetSearchRange(float& umin, float& umax, int px, int py,float mind,float maxd,
                                        ORB_SLAM2::KeyFrame* kf,ORB_SLAM2::KeyFrame* kf2)
{
    float fx = kf->fx;
    float cx = kf->cx;
    float fy = kf->fy;
    float cy = kf->cy;

    cv::Mat Rcw1 = kf->GetRotation();
    cv::Mat tcw1 = kf->GetTranslation();
    cv::Mat Rcw2 = kf2->GetRotation();
    cv::Mat tcw2 = kf2->GetTranslation();

    cv::Mat R21 = Rcw2*Rcw1.t();
    cv::Mat t21 = -Rcw2*Rcw1.t()*tcw1+tcw2;

    cv::Mat xp1=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);  // inverse project.    if has distortion, this code shoud fix
    cv::Mat xp2_min = R21*xp1*mind+t21;
    cv::Mat xp2_max = R21*xp1*maxd+t21;

    umin = fx*xp2_min.at<float>(0)/xp2_min.at<float>(2) + cx;
    umax = fx*xp2_max.at<float>(0)/xp2_max.at<float>(2) + cx;

    if (umin > umax){
        float temp = umax;
        umax = umin;
        umin = temp;
    }

    if(umin<0) umin = 0;
    if(umax<0) umax = 0;
    if(umin>kf->im_.cols ) umin = kf->im_.cols-1;
    if(umax>kf->im_.cols)  umax = kf->im_.cols-1;
}

bool ProbabilityMapping::ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val) {
    float num = (ha.depth - hb.depth)*(ha.depth - hb.depth);
    float chi_test = num / (ha.sigma*ha.sigma) + num / (hb.sigma*hb.sigma);
    if (chi_val)
        *chi_val = chi_test;
    return (chi_test < 5.99);  // 5.99 -> 95%
}

bool ProbabilityMapping::ChiTest(const float& a, const float& b, const float sigma_a,float sigma_b) {
    float num = (a - b)*(a - b);
    float chi_test = num / (sigma_a*sigma_a) + num / (sigma_b*sigma_b);
    return (chi_test < 5.99);  // 5.99 -> 95%
}

void ProbabilityMapping::GetFusion(const std::vector<std::pair <float,float> > supported, float& depth, float& sigma)
{
    size_t t = supported.size();
    float pjsj =0; // numerator
    float rsj =0; // denominator

    float min_sigma = supported[0].second;

    for(size_t i = 0; i< t; i++)
    {
        pjsj += supported[i].first / pow(supported[i].second, 2);
        rsj += 1 / pow(supported[i].second, 2);

        if (supported[i].second < min_sigma)
            min_sigma = supported[i].second;
    }

    depth = pjsj / rsj;
//    sigma = sqrt(1 / rsj);
    sigma = min_sigma;
}

void ProbabilityMapping::GetFusion(const std::vector<depthHo>& compatible_ho, depthHo& hypothesis, float* min_sigma) {
    hypothesis.depth = 0;
    hypothesis.sigma = 0;

    float temp_min_sigma = compatible_ho[0].sigma;
    float pjsj =0; // numerator
    float rsj =0; // denominator

    for (size_t j = 0; j < compatible_ho.size(); j++) {
        pjsj += compatible_ho[j].depth / pow(compatible_ho[j].sigma, 2);
        rsj += 1 / pow(compatible_ho[j].sigma, 2);
        if (pow(compatible_ho[j].sigma, 2) < pow(temp_min_sigma, 2)) {
            temp_min_sigma = compatible_ho[j].sigma;
        }
    }

    hypothesis.depth = pjsj / rsj;
    hypothesis.sigma = sqrt(1 / rsj);
    hypothesis.supported = true;

    if (min_sigma) {
        *min_sigma = temp_min_sigma;
    }
}

cv::Mat ProbabilityMapping::ComputeFundamental( ORB_SLAM2::KeyFrame *&pKF1,  ORB_SLAM2::KeyFrame *&pKF2) {
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = GetSkewSymmetricMatrix(t12);

    cv::Mat K1 = pKF1->GetCalibrationMatrix();
    cv::Mat K2 = pKF2->GetCalibrationMatrix();

    return K1.t().inv()*t12x*R12*K2.inv();
}

cv::Mat ProbabilityMapping::GetSkewSymmetricMatrix(const cv::Mat &v) {
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}



