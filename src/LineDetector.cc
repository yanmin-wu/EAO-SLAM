#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <random>
#include <vector>
#include <ctime>
#include "KeyFrame.h"
#include "MapPoint.h"

#include "LineDetector.h"
#include "Modeler.h"

#include <opencv2/core/eigen.hpp>
#include <Thirdparty/Line3Dpp/clustering.h>
#include <boost/filesystem.hpp>


int MIN_LINE_LENGTH = 10; //fixed here, can set to be related to image size
int MAX_LINE_LENGTH = 1000;
int INIT_DEPTH_COUNT = 3;
float MIN_SEGMENT_ANGLE = 30;
float E1 = 1.0;
float E2 = 1.5;

float lambda_a = 10.0;
float lambda_d = 0.02;
float sigma_limit = 0.02;
int line3D_covisN = 10;

/// Function prototype for DetectEdgesByED exported by EDLinesLib.a
LS *DetectLinesByED(unsigned char *srcImg, int width, int height, int *pNoLines);


LineDetector::LineDetector(){
    mAllLines = cv::Mat::zeros(0,6,CV_32F);
    mStrDateTime = CurrentDateTime();
}

void LineDetector::Reset(){
    mAllLines = cv::Mat::zeros(0,6,CV_32F);
    mStrDateTime = CurrentDateTime();
    clusters.clear();
    vpKFs_global.clear();
}

std::string LineDetector::CurrentDateTime(){
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H:%M:%S",timeinfo);
    std::string str(buffer);

    return str;
}

std::string LineDetector::GetStringDateTime() {
    return mStrDateTime;
}


void LineDetector::Summary()
{

    std::cout << std::endl;

    double sum_edge = 0.0;
    for(std::vector<double>::iterator it = time_edge.begin(); it != time_edge.end(); ++it)
        sum_edge += *it;
    std::cout << "Edge Drawing: " << sum_edge << "ms on " << time_edge.size() << " KFs, "
              << sum_edge/time_edge.size() << "ms/KF" << std::endl;


    double sum_edlines = 0.0;
    for(std::vector<double>::iterator it = time_edlines.begin(); it != time_edlines.end(); ++it)
        sum_edlines += *it;
    std::cout << "EDlines: " << sum_edlines << "ms on " << time_edlines.size() << " KFs, "
              << sum_edlines/time_edlines.size() << "ms/KF" << std::endl;


    std::cout << "Line3D: " << time_line3d[0] << "ms on " << (int)time_line3d[1] << " KFs, "
              << time_line3d[0]/time_line3d[1] << "ms/KF" << std::endl;


    std::cout << "Decoupled: " << time_decoupled[0] << "ms on " << (int)time_decoupled[1] << " KFs, "
              << time_decoupled[0]/time_decoupled[1] << "ms/KF" << std::endl;


    double sum_fitting = 0.0;
    for(std::vector<double>::iterator it = time_fitting.begin(); it != time_fitting.end(); ++it)
        sum_fitting += *it;
    std::cout << "Fitting: " << sum_fitting << "ms on " << time_fitting.size() << " KFs, "
              << sum_fitting/time_fitting.size() << "ms/KF" << std::endl;


    double sum_merging = 0.0;
    for(std::vector<double>::iterator it = time_merging.begin(); it != time_merging.end(); ++it)
        sum_merging += *it;
    std::cout << "Merging: " << sum_merging << "ms on " << time_merging.size() << " KFs, "
              << sum_merging/time_merging.size() << "ms/KF" << std::endl;

    std::cout << std::endl;


    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    std::string strFileName("results_line_segments/" + mStrDateTime + "/info.txt");
    std::ofstream fileOut(strFileName.c_str(), std::ios::out);
    if(!fileOut){
        std::cerr << "Failed to save running info" << std::endl;
        return;
    }

    fileOut << "Edge Drawing: " << sum_edge << "ms on " << time_edge.size() << " KFs, "
            << sum_edge/time_edge.size() << "ms/KF" << std::endl;

    fileOut << "EDlines: " << sum_edlines << "ms on " << time_edlines.size() << " KFs, "
            << sum_edlines/time_edlines.size() << "ms/KF" << std::endl;

    fileOut << "Line3D: " << time_line3d[0] << "ms on " << (int)time_line3d[1] << " KFs, "
            << time_line3d[0]/time_line3d[1] << "ms/KF" << std::endl;

    fileOut << "Decoupled: " << time_decoupled[0] << "ms on " << (int)time_decoupled[1] << " KFs, "
            << time_decoupled[0]/time_decoupled[1] << "ms/KF" << std::endl;

    fileOut << "Fitting: " << sum_fitting << "ms on " << time_fitting.size() << " KFs, "
            << sum_fitting/time_fitting.size() << "ms/KF" << std::endl;

    fileOut << "Merging: " << sum_merging << "ms on " << time_merging.size() << " KFs, "
            << sum_merging/time_merging.size() << "ms/KF" << std::endl;

    fileOut << std::endl;

    fileOut << "Parameters:" << std::endl;
    fileOut << "MIN_LINE_LENGTH = " << MIN_LINE_LENGTH << std::endl;
    fileOut << "MAX_LINE_LENGTH = " << MAX_LINE_LENGTH << std::endl;
    fileOut << "INIT_DEPTH_COUNT = " << INIT_DEPTH_COUNT << std::endl;
    fileOut << "MIN_SEGMENT_ANGLE = " << MIN_SEGMENT_ANGLE << std::endl;
    fileOut << "E1 = " << E1 << std::endl;
    fileOut << "E2 = " << E2 << std::endl;
    fileOut << "lambda_a = " << lambda_a << std::endl;
    fileOut << "lambda_d = " << lambda_d << std::endl;
    fileOut << "sigma_limit = " << sigma_limit << std::endl;

    if (time_modeling.size() >= 2) {
        fileOut << std::endl << "Modeling: " << time_modeling[0] << "ms on " << (int)time_modeling[1] << " KFs, "
                                             << time_modeling[0]/time_modeling[1] << "ms/KF" << std::endl;
    }

    fileOut.flush();
    fileOut.close();
    std::cout << "Saved running info" << std::endl;

}

void LineDetector::RunLine3Dpp(std::vector<ORB_SLAM2::KeyFrame *> vpKFs) {
    struct timespec start, finish;
    double duration;

    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &start);

    // Line3D reconstruct lines
    L3DPP::Line3D* Line3D = new L3DPP::Line3D("results_line_segments/" + mStrDateTime, false, 1500, 3000, false, false);

    int usedKF = 0;

    for (size_t indKF = 0; indKF < vpKFs.size(); indKF++) {
        ORB_SLAM2::KeyFrame* kf = vpKFs[indKF];
        kf->SetNotEraseSemiDense();
        if( kf->isBad() || !kf->semidense_flag_ ) {
            kf->SetEraseSemiDense();
            continue;
        }
        // use covisN good neighbor kfs
        std::vector<ORB_SLAM2::KeyFrame*> closestMatchesAll = kf->GetVectorCovisibleKeyFrames();
        std::vector<ORB_SLAM2::KeyFrame*> closestMatches;
        for (size_t idxCov = 0; idxCov < closestMatchesAll.size(); idxCov++){
            if ((int)closestMatches.size() >= line3D_covisN)
                break;
            ORB_SLAM2::KeyFrame* kfCM = closestMatchesAll[idxCov];
            kfCM->SetNotEraseSemiDense();
            closestMatches.push_back(kfCM);
        }

        std::list<unsigned int> lNeighborId;
        for (size_t idxCov = 0; idxCov < closestMatches.size(); idxCov++){
            lNeighborId.push_back(closestMatches[idxCov]->mnId);
        }

        std::vector<cv::Vec4f> line_segments;
        for (int idxRow = 0; idxRow < kf->mLines.rows; idxRow++){
            line_segments.push_back(cv::Vec4f(kf->mLines.at<float>(idxRow,0),kf->mLines.at<float>(idxRow,1),kf->mLines.at<float>(idxRow,2),kf->mLines.at<float>(idxRow,3)));
        }

        Eigen::Matrix3d K;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        cv::cv2eigen(kf->mK, K);
        cv::cv2eigen(kf->GetPose().colRange(0,3).clone(), R);
        cv::cv2eigen(kf->GetPose().colRange(3,4).clone(), t);


        Line3D->addImage(kf->mnId, kf->im_, K, R, t, kf->ComputeSceneMedianDepth(2), lNeighborId, line_segments);

        usedKF++;

        for (size_t idxCov = 0; idxCov < closestMatches.size(); idxCov++){
            closestMatches[idxCov]->SetEraseSemiDense();
        }

        kf->SetEraseSemiDense();
    }

    Line3D->matchImages();

    Line3D->reconstruct3Dlines();

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    duration *= 1000.0;
    std::cout << "line3d++ reconstruction took total: "<< duration << "ms  avg:" << duration/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;

    time_line3d.push_back(duration);
    time_line3d.push_back((double)usedKF);

    Line3D->saveResultAsOBJ("results_line_segments/" + mStrDateTime);

}


void LineDetector::LineFittingEDLinesOffline(std::vector<ORB_SLAM2::KeyFrame *> vpKFs) {
    struct timespec start, finish;
    double duration;


    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &start);

    // save line segments
    std::string strFileName("results_line_segments/" + mStrDateTime + "/line_segments_edlines.obj");
    std::ofstream fileOut(strFileName.c_str(), std::ios::out);
    if(!fileOut){
        std::cerr << "Failed to open stream to file: " << strFileName << std::endl;
        return;
    }

    int lineID = 0;
    int pointID = 1;
    std::map<int,int> lines2points;

    for (size_t indKF = 0; indKF < vpKFs.size(); indKF++) {
        ORB_SLAM2::KeyFrame* kf = vpKFs[indKF];
        kf->SetNotEraseSemiDense();
        if( kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
            kf->SetEraseSemiDense();
            continue;
        }

        for (int k = 0; k < kf->mLines.rows; k++){
            cv::Point2f s(kf->mLines.at<float>(k,0), kf->mLines.at<float>(k,1));
            cv::Point2f e(kf->mLines.at<float>(k,2), kf->mLines.at<float>(k,3));

            float la = s.y - e.y;
            float lb = e.x - s.x;
            float lc = (s.x - e.x) * s.y + (e.y - s.y) * s.x;

            cv::LineIterator it(kf->depth_map_checked_, s, e, 4);

            // get confident points on the line
            std::vector<cv::Point> points;

            for (int i = 0; i < it.count; i++, it++) {
                cv::Point curr = it.pos();
                if (curr.x < kf->mnMinX+1 || curr.x >=  kf->mnMaxX-1) continue;
                if (curr.y < kf->mnMinY+1 || curr.y >=  kf->mnMaxY-1) continue;

                cv::Point2f curr2f(curr.x, curr.y);
                // avoid points on the other direction around s
                if (cv::norm(curr2f-e) > cv::norm(s-e)) continue;

                float invz = kf->depth_map_checked_.at<float>(curr.y,curr.x);
                float sigma = kf->depth_sigma_.at<float>(curr.y,curr.x);
                if (invz > 0.000001 && sigma < sigma_limit) {
                    points.push_back(curr);
                }
            }

            if ((int)points.size() < INIT_DEPTH_COUNT)
                continue;

            if((float)points.size()/(float)it.count < (float)INIT_DEPTH_COUNT / (float)MIN_LINE_LENGTH)
                continue;

            vector<bool> inlier((int)points.size(), false);
            int maxCount = 0;

            for (int ransac = 0; ransac < (int)points.size(); ransac++){

                // generate random set of index
                // from: https://stackoverflow.com/questions/2394246/algorithm-to-select-a-single-random-combination-of-values

                // RNG for RANSAC
                std::random_device rd;
                std::mt19937 gen(rd());
                // random from 1 to N
                std::uniform_int_distribution<> dis1(1, (int)points.size()-1);
                int r1 = dis1(gen);
                std::uniform_int_distribution<> dis2(1, (int)points.size());
                int r2 = dis2(gen);
                if (r2 == r1)
                    r2 = (int)points.size();

                // get index 0 to N-1
                r1--; r2--;

                cv::Point2f p1, p2;
                ClosestPointOnLine(la, lb, lc, points[r1].x, points[r1].y, p1.x, p1.y);
                ClosestPointOnLine(la, lb, lc, points[r2].x, points[r2].y, p2.x, p2.y);

                float d1 = (float)cv::norm(p1 - s);
                float d2 = (float)cv::norm(p2 - s);

                float z1 = kf->depth_map_checked_.at<float>(points[r1].y, points[r1].x);
                float z2 = kf->depth_map_checked_.at<float>(points[r2].y, points[r2].x);
                // convert depth to pixel scale using the average of fx and fy
                z1 *= (kf->fx + kf->fy)/2;
                z2 *= (kf->fx + kf->fy)/2;

                float u1 = (z2 - z1)/(d2 - d1);
                float u2 = (d2*z1 - d1*z2)/(d2 - d1);

                vector<bool> currInlier((int)points.size(), false);
                int currCount = 0;

                for (int i = 0; i < (int)points.size(); i++) {
                    cv::Point2f curr;
                    ClosestPointOnLine(la, lb, lc, points[i].x, points[i].y, curr.x, curr.y);

                    float t = (float)cv::norm(curr - s);
                    float z = kf->depth_map_checked_.at<float>(points[i].y, points[i].x);
                    // convert depth to pixel scale using the average of fx and fy
                    z *= (kf->fx + kf->fy)/2;

                    float dist = std::abs(u1*t-z+u2)/std::sqrt(u1*u1+1);

                    if (dist < E2){
                        currCount++;
                        currInlier[i] = true;
                    }
                }

                if (currCount > maxCount){
                    maxCount = currCount;
                    inlier.swap(currInlier);
                }
            }

            // z=u1*d+u2, where d is distance to start point, z is depth
            cv::Mat A((int)points.size(), 2, CV_32F, cv::Scalar(0));
            cv::Mat b((int)points.size(), 1, CV_32F, cv::Scalar(0));
            // u=[u1,u2]
            cv::Mat u(2, 1, CV_32F, cv::Scalar(0));

            for (int i = 0; i < (int)points.size(); i++){

                // only use inlier
                if (inlier[i] == false) continue;

                cv::Point curr = points[i];
                float invz = kf->depth_map_checked_.at<float>(curr.y,curr.x);

                cv::Point2f currProj;
                ClosestPointOnLine(la, lb, lc, curr.x, curr.y, currProj.x, currProj.y);
                float d = (float)cv::norm(currProj - s);

                float weight = 1;
                A.at<float>(i,0) = weight * d;
                A.at<float>(i,1) = weight;
                b.at<float>(i) = weight * 1/invz;
                // convert depth to pixel scale using the average of fx and fy
                b.at<float>(i) *= (kf->fx + kf->fy)/2;

            }

            // solve SVD
            cv::solve(A, b, u, cv::DECOMP_SVD);

            // recover 3d position for start and end point
            cv::Mat Twc = kf->GetPoseInverse();

            float Zs = u.at<float>(1);
            // back to world scale
            Zs /= (kf->fx + kf->fy)/2;
            float Xs = Zs * (s.x - kf->cx) / kf->fx;
            float Ys = Zs * (s.y - kf->cy) / kf->fy;

            float Ze = u.at<float>(0) * (float)cv::norm(e-s) + u.at<float>(1);
            // back to world scale
            Ze /= (kf->fx + kf->fy)/2;
            float Xe = Ze * (e.x - kf->cx) / kf->fx;
            float Ye = Ze * (e.y - kf->cy) / kf->fy;


            // avoid line segments shoot outward the camera center
            cv::Mat Pcs3 = (cv::Mat_<float>(3, 1) << Xs, Ys, Zs);
            cv::Mat Pce3 = (cv::Mat_<float>(3, 1) << Xe, Ye, Ze);

            cv::Mat diffxyz = Pce3 - Pcs3;
            double cosS = diffxyz.dot(Pcs3) / cv::norm(diffxyz) / cv::norm(Pcs3);
            double cosE = diffxyz.dot(Pce3) / cv::norm(diffxyz) / cv::norm(Pce3);
            double angleS = std::acos(std::abs(cosS)) * 180 / CV_PI;
            double angleE = std::acos(std::abs(cosE)) * 180 / CV_PI;

            if (angleS > MIN_SEGMENT_ANGLE && angleE > MIN_SEGMENT_ANGLE) {

                cv::Mat Pcs = (cv::Mat_<float>(4, 1) << Xs, Ys, Zs, 1); // point in camera frame.
                cv::Mat Pws = Twc * Pcs;
                cv::Mat Pce = (cv::Mat_<float>(4, 1) << Xe, Ye, Ze, 1); // point in camera frame.
                cv::Mat Pwe = Twc * Pce;

                fileOut << "v " << Pws.at<float>(0) << " " << Pws.at<float>(1) << " " << Pws.at<float>(2) << std::endl;
                fileOut << "v " << Pwe.at<float>(0) << " " << Pwe.at<float>(1) << " " << Pwe.at<float>(2) << std::endl;

                lines2points[lineID] = pointID;

                lineID++;
                pointID += 2;
            }
        }

        kf->SetEraseSemiDense();
    }


    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    duration *= 1000.0;
    std::cout << "svd line reconstruction with EDLines took total: "<< duration << "ms  avg:" << duration/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;

    time_decoupled.push_back(duration);
    time_decoupled.push_back(vpKFs.size());

    std::map<int,int>::const_iterator it = lines2points.begin();
    for(; it!=lines2points.end(); ++it)
    {
        fileOut << "l " << it->second << " " << it->second+1 << std::endl;
    }

    fileOut.flush();
    fileOut.close();
    std::cout << "saved EDLines line segment cloud" << std::endl;

}


void LineDetector::LineFittingOffline(std::vector<ORB_SLAM2::KeyFrame *> vpKFs, Modeler* pModeler) {
    struct timespec start, finish, start_part, finish_part;
    double duration, duration_fit, duration_cluster, temp_duration_part;
    duration_fit = 0.0;
    duration_cluster = 0.0;

    clock_gettime(CLOCK_MONOTONIC, &start);

    for (size_t indKF = 0; indKF < vpKFs.size(); indKF++) {
        ORB_SLAM2::KeyFrame* kf = vpKFs[indKF];
        kf->SetNotEraseSemiDense();
        if( kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
            kf->SetEraseSemiDense();
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &start_part);

        LineFitting(kf);

        clock_gettime(CLOCK_MONOTONIC, &finish_part);
        temp_duration_part = ( finish_part.tv_sec - start_part.tv_sec );
        temp_duration_part += ( finish_part.tv_nsec - start_part.tv_nsec ) / 1000000000.0;
        temp_duration_part *= 1000.0;
        duration_fit += temp_duration_part;

        std::cout << "mLines3D.rows: " <<  kf->mLines3D.rows << std::endl;

        clock_gettime(CLOCK_MONOTONIC, &start_part);

        MergeLines(kf, pModeler);

        clock_gettime(CLOCK_MONOTONIC, &finish_part);
        temp_duration_part = ( finish_part.tv_sec - start_part.tv_sec );
        temp_duration_part += ( finish_part.tv_nsec - start_part.tv_nsec ) / 1000000000.0;
        temp_duration_part *= 1000.0;
        duration_cluster += temp_duration_part;

        kf->SetEraseSemiDense();
    }

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    duration *= 1000.0;
    std::cout << "svd line reconstruction took total: "<< duration << "ms  avg:" << duration/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;
    std::cout << "svd line fitting took total: "<< duration_fit << "ms  avg:" << duration_fit/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;
    std::cout << "svd line merging took total: "<< duration_cluster << "ms  avg:" << duration_cluster/vpKFs.size() << "ms  #KF:" << vpKFs.size() << std::endl;

}

void LineDetector::SaveAllLineSegments() {

    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    // save line segments
    std::string strFileName2Fold("results_line_segments/" + mStrDateTime + "/line_segments.obj");
    std::ofstream fileOut2Fold(strFileName2Fold.c_str(), std::ios::out);
    if(!fileOut2Fold){
        std::cerr << "Failed to open stream to file: " << strFileName2Fold << std::endl;
        return;
    }

    int lineID2Fold = 0;
    int pointID2Fold = 1;
    std::map<int,int> lines2points2Fold;

    for (int k = 0; k < mAllLines.rows; k++){
        fileOut2Fold << "v " << mAllLines.at<float>(k,0) << " " << mAllLines.at<float>(k,1) << " " << mAllLines.at<float>(k,2) << std::endl;
        fileOut2Fold << "v " << mAllLines.at<float>(k,3) << " " << mAllLines.at<float>(k,4) << " " << mAllLines.at<float>(k,5) << std::endl;

        lines2points2Fold[lineID2Fold] = pointID2Fold;

        lineID2Fold++;
        pointID2Fold+=2;
    }

    std::map<int,int>::const_iterator it2Fold = lines2points2Fold.begin();
    for(; it2Fold!=lines2points2Fold.end(); ++it2Fold)
    {
        fileOut2Fold << "l " << it2Fold->second << " " << it2Fold->second+1 << std::endl;
    }

    fileOut2Fold.flush();
    fileOut2Fold.close();
    std::cout << "saved line segment cloud" << std::endl;

}


void LineDetector::ClosestPointOnLine(float a, float b, float c, int x, int y, float& cx, float& cy)
{
    cx = (b*(b*x - a*y) - a*c)/(a*a + b*b);
    cy = (a*(-b*x + a*y) - b*c)/(a*a + b*b);
}


int LineDetector::CountDepth(Pixel *pixelChain, int length, ORB_SLAM2::KeyFrame* kf)
{
    int count = 0;
    for (int i = 0; i < length; i++) {
        int x = pixelChain[i].c;
        int y = pixelChain[i].r;
        if (kf->depth_map_checked_.at<float>(y,x) > 0.000001
            && kf->depth_sigma_.at<float>(y,x) < sigma_limit) {
            count++;
        }
    }
    return count;
}


void LineDetector::LeastSquaresLineFit(Pixel* pixelChain, int length, float& u1, float& u2, float& u3, float& lineFitError)
{

    // SVD solve A*u=b, least square
    // u1*x + u2*y + u3 = 0
    cv::Mat A(length, 3, CV_32F, cv::Scalar(0));
    // u=[u1,u2,u3]
    cv::Mat u(3, 1, CV_32F, cv::Scalar(0));

    for (int i = 0; i < length; i++){
        int x = pixelChain[i].c;
        int y = pixelChain[i].r;

        A.at<float>(i,0) = x;
        A.at<float>(i,1) = y;
        A.at<float>(i,2) = 1;
    }

    cv::SVD::solveZ(A,u);

    u1 = u.at<float>(0);
    u2 = u.at<float>(1);
    u3 = u.at<float>(2);

    lineFitError = (float)cv::norm(A*u);
}


void LineDetector::LeastSquaresDepthFit(Pixel* pixelChain, int length, float la, float lb, float lc, float& u1, float& u2, float& depthFitError, ORB_SLAM2::KeyFrame* kf)
{

    // z=u1*d+u2, where d is distance to start point, z is depth
    cv::Mat A(length, 2, CV_32F, cv::Scalar(0));

    cv::Mat b(length, 1, CV_32F, cv::Scalar(0));

    // u=[u1,u2]
    cv::Mat u(2, 1, CV_32F, cv::Scalar(0));

    // start point of pixel chain
    cv::Point2f s;
    ClosestPointOnLine(la, lb, lc, pixelChain[0].c, pixelChain[0].r, s.x, s.y);

    for (int i = 0; i < length; i++){
        int x = pixelChain[i].c;
        int y = pixelChain[i].r;

        float invz = kf->depth_map_checked_.at<float>(y,x);
        float sigma = kf->depth_sigma_.at<float>(y,x);
        if (invz > 0.000001 && sigma < sigma_limit){
            cv::Point2f curr;
            ClosestPointOnLine(la, lb, lc, x, y, curr.x, curr.y);
            float d = (float)cv::norm(curr - s);

            float weight = 1;
            A.at<float>(i,0) = weight * d;
            A.at<float>(i,1) = weight;

            b.at<float>(i) = weight * 1/invz;
            // convert depth to pixel scale using the average of fx and fy
            b.at<float>(i) *= (kf->fx + kf->fy)/2;
        }
    }

    cv::solve(A, b, u, cv::DECOMP_SVD);

    u1 = u.at<float>(0);
    u2 = u.at<float>(1);

    depthFitError = (float)cv::norm(A*u, b);

}

float LineDetector::ComputePointDistance2Line(float a, float b, float c, Pixel pixel)
{
    int x = pixel.c;
    int y = pixel.r;

    return std::abs(a*x+b*y+c)/std::sqrt(a*a+b*b);
}

float LineDetector::ComputePointDepth2Line(float a, float b, float c, float alpha, float beta, Pixel *pixelChain, Pixel pixel, ORB_SLAM2::KeyFrame* kf)
{
    int x = pixel.c;
    int y = pixel.r;

    float z = 1 / kf->depth_map_checked_.at<float>(y,x);

    if (z < 0.000001){
        return -1;
    }

    float sigma = kf->depth_sigma_.at<float>(y,x);

    if (sigma > sigma_limit){
        return -1;
    }

    cv::Point2f s;
    ClosestPointOnLine(a, b, c, pixelChain[0].c, pixelChain[0].r, s.x, s.y);
    cv::Point2f curr;
    ClosestPointOnLine(a, b, c, x, y, curr.x, curr.y);
    float t = (float)cv::norm(curr - s);
    // convert depth to pixel scale using the average of fx and fy
    z *= (kf->fx + kf->fy)/2;

    return std::abs(alpha*t-z+beta)/std::sqrt(alpha*alpha+1);
}


void LineDetector::LineFit(Pixel *pixelChain, int noPixels, ORB_SLAM2::KeyFrame* kf)
{
    float lineFitError = std::numeric_limits<float>::infinity();
    float depthFitError = std::numeric_limits<float>::infinity();
    float a = 0; float b = 0; float c = 0; // line equation: ax + by + c = 0
    float alpha = 0; float beta = 0; // line equation: z = alpha * d + beta

    int DEPTH_CHECK_INTERVAL = MIN_LINE_LENGTH;

    int initLength = MIN_LINE_LENGTH;

    while (noPixels > initLength && initLength < MAX_LINE_LENGTH){
        if (CountDepth(pixelChain, 1, kf) < 1){
            pixelChain++;
            noPixels--;
            continue;
        }
        if (CountDepth(pixelChain, initLength, kf) < INIT_DEPTH_COUNT){
            pixelChain++;
            noPixels--;
            continue;
        }

        LeastSquaresLineFit(pixelChain, initLength, a, b, c, lineFitError);
        LeastSquaresDepthFit(pixelChain, initLength, a, b, c, alpha, beta, depthFitError, kf);
        if (lineFitError <= 1.0 && depthFitError <= 1.0) break;

        pixelChain++;
        noPixels--;
    }

    if (lineFitError > E1) return;

    if (depthFitError > E2) return;

    int interval = 0;
    int lineLen = initLength;
    while (lineLen < MAX_LINE_LENGTH && lineLen < noPixels){
        float dist = ComputePointDistance2Line(a, b, c, pixelChain[lineLen]);
        if (dist > E1) break;

        lineLen++;
        interval++;

        if (interval >= DEPTH_CHECK_INTERVAL){
            interval = 0;
            if (CountDepth(pixelChain+lineLen-DEPTH_CHECK_INTERVAL, DEPTH_CHECK_INTERVAL, kf) < 1){
                lineLen -= DEPTH_CHECK_INTERVAL;
                break;
            }
            int prev = lineLen - DEPTH_CHECK_INTERVAL;
            bool stop = false;
            for (int i = prev; i < lineLen; i++) {
                float dept = ComputePointDepth2Line(a, b, c, alpha, beta, pixelChain, pixelChain[i], kf);
                if (dept > E2){
                    lineLen = prev;
                    stop = true;
                    break;
                }
                if (dept >= 0.0){
                    prev = i;
                }
            }
            if (stop) break;

        }

    }

    LeastSquaresLineFit(pixelChain, lineLen, a, b, c, lineFitError);


    if (CountDepth(pixelChain, lineLen, kf) / (float)lineLen > (float)INIT_DEPTH_COUNT / (float)MIN_LINE_LENGTH)
    {
        LeastSquaresDepthFit(pixelChain, lineLen, a, b, c, alpha, beta, depthFitError, kf);


        // get subpixel coordinate of start and end on line
        cv::Point2f s;
        ClosestPointOnLine(a, b, c, pixelChain[0].c, pixelChain[0].r, s.x, s.y);
        cv::Point2f e;
        ClosestPointOnLine(a, b, c, pixelChain[lineLen - 1].c, pixelChain[lineLen - 1].r, e.x, e.y);

        // recover 3d position for start and end point
        cv::Mat Twc = kf->GetPoseInverse();

        float Zs = beta;
        // back to world scale
        Zs /= (kf->fx + kf->fy) / 2;
        float Xs = Zs * (s.x - kf->cx) / kf->fx;
        float Ys = Zs * (s.y - kf->cy) / kf->fy;

        float Ze = alpha * (float) cv::norm(e - s) + beta;
        // back to world scale
        Ze /= (kf->fx + kf->fy) / 2;
        float Xe = Ze * (e.x - kf->cx) / kf->fx;
        float Ye = Ze * (e.y - kf->cy) / kf->fy;

        // avoid line segments shoot outward the camera center
        cv::Mat Pcs3 = (cv::Mat_<float>(3, 1) << Xs, Ys, Zs);
        cv::Mat Pce3 = (cv::Mat_<float>(3, 1) << Xe, Ye, Ze);

        cv::Mat diffxyz = Pce3 - Pcs3;
        double cosS = diffxyz.dot(Pcs3) / cv::norm(diffxyz) / cv::norm(Pcs3);
        double cosE = diffxyz.dot(Pce3) / cv::norm(diffxyz) / cv::norm(Pce3);
        double angleS = std::acos(std::abs(cosS)) * 180 / CV_PI;
        double angleE = std::acos(std::abs(cosE)) * 180 / CV_PI;

        if (angleS > MIN_SEGMENT_ANGLE && angleE > MIN_SEGMENT_ANGLE) {

            cv::Mat Pcs = (cv::Mat_<float>(4, 1) << Xs, Ys, Zs, 1); // point in camera frame.
            cv::Mat Pws = Twc * Pcs;
            cv::Mat Pce = (cv::Mat_<float>(4, 1) << Xe, Ye, Ze, 1); // point in camera frame.
            cv::Mat Pwe = Twc * Pce;

            cv::Mat line = (cv::Mat_<float>(1, 6) << Pws.at<float>(0), Pws.at<float>(1), Pws.at<float>(2),
                    Pwe.at<float>(0), Pwe.at<float>(1), Pwe.at<float>(2));

            cv::Mat line2D = (cv::Mat_<float>(1, 4) << s.x, s.y, e.x, e.y);

            kf->mLines3D.push_back(line);
            kf->mLinesSeg.push_back(line2D);

        }
    }

    LineFit(pixelChain+lineLen, noPixels-lineLen, kf);

}


void LineDetector::DetectEdgeMap(ORB_SLAM2::KeyFrame* kf) {

    struct timespec start, finish;
    double duration;

    clock_gettime(CLOCK_MONOTONIC, &start);

    cv::Mat im = kf->GetImage();
    int width, height;
    unsigned char *srcImg;

    width = im.size().width;
    height = im.size().height;
    srcImg = im.data;

    EdgeMap *map = DetectEdgesByED(srcImg, width, height, SOBEL_OPERATOR, 36, 8, 1.0);

    for (int i=0; i<map->noSegments; i++){
        for (int j=0; j<map->segments[i].noPixels; j++){
            int r = map->segments[i].pixels[j].r;
            int c = map->segments[i].pixels[j].c;

            kf->mEdgeIndex.at<int>(r,c) = i;

        } //end-for
    } //end-for


    kf->mEdgeMap = map;

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = (finish.tv_sec - start.tv_sec);
    duration += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    std::cout << "edge detection took: " << duration << "s" << std::endl;
    std::cout << "#edge chains: " << map->noSegments << std::endl;

    time_edge.push_back(duration*1000);

}


void LineDetector::LineFitting(ORB_SLAM2::KeyFrame* kf)
{
    struct timespec start, finish;
    double duration;

    clock_gettime(CLOCK_MONOTONIC, &start);

    if(kf->mEdgeMap == NULL){
        std::cerr << "error: no edge map" << std::endl;
        return;
    }

    EdgeMap *map = kf->mEdgeMap;

    for (int i=0; i<map->noSegments; i++){

        LineFit(map->segments[i].pixels, map->segments[i].noPixels, kf);

    } //end-for

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    std::cout << "3d reconstruction line segments took: "<< duration << "s" << std::endl;
    std::cout << "#segments: " << kf->mLines3D.rows << std::endl;

    time_fitting.push_back(duration*1000);

}


void LineDetector::MergeLines(ORB_SLAM2::KeyFrame* kf, Modeler* pModeler)
{

    struct timespec start, finish;
    double duration;

    clock_gettime(CLOCK_MONOTONIC, &start);

    for (int i = 0; i < kf->mLines3D.rows; i++) {
        cv::Mat line = (cv::Mat_<float>(1, 6) << kf->mLines3D.at<float>(i, 0),
                kf->mLines3D.at<float>(i, 1),
                kf->mLines3D.at<float>(i, 2),
                kf->mLines3D.at<float>(i, 3),
                kf->mLines3D.at<float>(i, 4),
                kf->mLines3D.at<float>(i, 5));

        bool clustered = false;

        for (int j = 0; j < (int)clusters.size(); j++) {
            if (CLoseEnough(clusters[j], line)) {
                clusters[j].segments.push_back(line);
                UpdateCluster(clusters[j]);
                clustered = true;
                break;
            }
        }

        if (!clustered) {
            Cluster cluster;
            cluster.segments.push_back(line);
            UpdateCluster(cluster);
            cluster.index = clusters.size();
            clusters.push_back(cluster);
        }

        mAllLines.push_back(line);
    }

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    std::cout << "merge line segments took: "<< duration << "s" << std::endl;
    std::cout << "#clusters: " << clusters.size() << std::endl;

    time_merging.push_back(duration*1000);

}

bool LineDetector::CLoseEnough(Cluster c, cv::Mat l)
{

    cv::Mat s1 = c.centerSegment.colRange(0,3);
    cv::Mat e1 = c.centerSegment.colRange(3,6);

    cv::Mat s2 = l.colRange(0,3);
    cv::Mat e2 = l.colRange(3,6);

    // compute angular similarity
    cv::Mat dir1 = (e1 - s1) / cv::norm(e1 - s1);
    cv::Mat dir2 = (e2 - s2) / cv::norm(e2 - s2);

    float dot_p = (float) dir1.dot(dir2);
    float angle = acos(fmax(fmin(dot_p, 1.0f), -1.0f)) / (float) M_PI * 180.0f;
    if (angle > 90.0f) {
        angle = 180.0f - angle;
    }

    if (angle > lambda_a)
        return false;


    // compute distance
    float ds2 = (float)cv::norm((s1-s2) - dir1*(dir1.dot(s1-s2)));
    float de2 = (float)cv::norm((s1-e2) - dir1*(dir1.dot(s1-e2)));
    float mind2 = std::min(ds2, de2);

    if (mind2 < lambda_d){

        cv::Mat diff11 = s1-s2;
        cv::Mat diff12 = s1-e2;
        cv::Mat diff21 = e1-s2;
        cv::Mat diff22 = e1-e2;

        float d11 = (float)cv::norm(diff11);
        float d12 = (float)cv::norm(diff12);
        float d21 = (float)cv::norm(diff21);
        float d22 = (float)cv::norm(diff22);

        float len = (float)cv::norm(s1-e1);

        if (d11 + d21 < len + lambda_d)
            return true;
        if (d12 + d22 < len + lambda_d)
            return true;

    }

    return false;

}


void LineDetector::UpdateCluster(Cluster& c)
{
    if (c.segments.rows < 1){
        return;
    } else if (c.segments.rows == 1) {
        for (int i = 0; i < 6; i++) {
            c.centerSegment.at<float>(0, i) = c.segments.at<float>(0, i);
        }
        return;
    }  else if (c.segments.rows == 2) {
        for (int i = 0; i < 6; i++) {
            c.centerSegment.at<float>(0, i) = (c.segments.at<float>(0, i) + c.segments.at<float>(0, i)) / 2;
        }
        return;
    } else {
        cv::Mat A = cv::Mat::zeros(c.segments.rows*2, 3, CV_32F);
        for (int i = 0; i < c.segments.rows; i++){
            A.at<float>(2*i,0) = c.segments.at<float>(i,0);
            A.at<float>(2*i,1) = c.segments.at<float>(i,1);
            A.at<float>(2*i,2) = c.segments.at<float>(i,2);
            A.at<float>(2*i+1,0) = c.segments.at<float>(i,3);
            A.at<float>(2*i+1,1) = c.segments.at<float>(i,4);
            A.at<float>(2*i+1,2) = c.segments.at<float>(i,5);
        }

        cv::Mat center;
        cv::reduce(A, center, 0, CV_REDUCE_AVG);

        cv::Mat B = A.clone();
        for (int i = 0; i < B.rows; i++){
            B.at<float>(i,0) -= center.at<float>(0,0);
            B.at<float>(i,1) -= center.at<float>(0,1);
            B.at<float>(i,2) -= center.at<float>(0,2);
        }

        cv::SVD svd(B.t());

        int maxw = 0;
        float lastw = svd.w.at<float>(0);
        for (int indw = 1; indw < svd.w.rows; indw++){
            if (svd.w.at<float>(indw) > lastw) {
                maxw = indw;
                lastw = svd.w.at<float>(indw);
            }
        }
        cv::Mat dir = svd.u.col(maxw);
        dir = dir.t();


        float maxT = 0;
        float minT = 0;

        for (int i = 0; i < A.rows; i++) {
            cv::Mat diff = A.row(i) - center;
            float t = (float)diff.dot(dir);
            if (t > maxT) maxT = t;
            if (t < minT) minT = t;
        }

        cv::Mat s = center + dir * maxT;
        cv::Mat e = center + dir * minT;

        c.centerSegment.at<float>(0,0) = s.at<float>(0,0);
        c.centerSegment.at<float>(0,1) = s.at<float>(0,1);
        c.centerSegment.at<float>(0,2) = s.at<float>(0,2);
        c.centerSegment.at<float>(0,3) = e.at<float>(0,0);
        c.centerSegment.at<float>(0,4) = e.at<float>(0,1);
        c.centerSegment.at<float>(0,5) = e.at<float>(0,2);

        return;
    }
}


void LineDetector::SaveClusteredSegments()
{
    struct timespec start, finish;
    double duration;

    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &start);

    // save line segments
    std::string strFileName("results_line_segments/" + mStrDateTime + "/line_segments_clustered_incr.obj");
    std::ofstream fileOut(strFileName.c_str(), std::ios::out);
    if(!fileOut){
        std::cerr << "Failed to save points on line" << std::endl;
        return;
    }

    int lineID = 0;
    int pointID = 1;
    std::map<int,int> lines2points;

    for (size_t indCL = 0; indCL < clusters.size(); indCL++) {

        if (clusters[indCL].segments.rows < 3)
            continue;

        fileOut << "v " << clusters[indCL].centerSegment.at<float>(0,0) << " "
                << clusters[indCL].centerSegment.at<float>(0,1) << " "
                << clusters[indCL].centerSegment.at<float>(0,2) << std::endl;
        fileOut << "v " << clusters[indCL].centerSegment.at<float>(0,3) << " "
                << clusters[indCL].centerSegment.at<float>(0,4) << " "
                << clusters[indCL].centerSegment.at<float>(0,5) << std::endl;

        lines2points[lineID] = pointID;

        lineID++;
        pointID += 2;

    }

    std::map<int,int>::const_iterator it = lines2points.begin();
    for(; it!=lines2points.end(); ++it)
    {
        fileOut << "l " << it->second << " " << it->second+1 << std::endl;
    }

    fileOut.flush();
    fileOut.close();
    std::cout << "saved incremental clustered line segment cloud" << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    std::cout << "saving incremental clustered line reconstruction took: "<< duration << "s" << std::endl;

}


void LineDetector::DetectLineSegments(ORB_SLAM2::KeyFrame* kf) {

    struct timespec start, finish;
    double duration;
    clock_gettime(CLOCK_MONOTONIC, &start);

    cv::Mat im = kf->GetImage();
    int width, height;
    unsigned char *srcImg;
    int noLines;

    width = im.size().width;
    height = im.size().height;
    srcImg = im.data;

    LS *lines = DetectLinesByED(srcImg, width, height, &noLines);

    cv::Mat lineMat = cv::Mat::zeros(noLines,4,CV_32F);
    for (int k = 0; k < noLines; k++) {
        lineMat.at<float>(k,0) = (float)lines[k].sx;
        lineMat.at<float>(k,1) = (float)lines[k].sy;
        lineMat.at<float>(k,2) = (float)lines[k].ex;
        lineMat.at<float>(k,3) = (float)lines[k].ey;
    }

    delete lines;

    kf->mLines.push_back(lineMat);


    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    duration *= 1000.0;


    time_edlines.push_back(duration);


    for (int k = 0; k < kf->mLines.rows; k++){
        cv::Point2i s((int)kf->mLines.at<float>(k,0), (int)kf->mLines.at<float>(k,1));
        cv::Point2i e((int)kf->mLines.at<float>(k,2), (int)kf->mLines.at<float>(k,3));

        cv::LineIterator it(kf->mLineIndex, s, e);
        for (int i = 0; i < it.count; i++, ++it){
            cv::Point curr = it.pos();
            if (curr.x < kf->mnMinX+1 || curr.x >=  kf->mnMaxX-1) continue;
            if (curr.y < kf->mnMinY+1 || curr.y >=  kf->mnMaxY-1) continue;

            if (kf->mLineIndex.at<int>(curr.y,curr.x) < 0){
                kf->mLineIndex.at<int>(curr.y,curr.x) = k;
            }

        }

    }



}





