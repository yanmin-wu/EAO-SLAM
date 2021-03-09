//
// Created by shida3 on 8/24/17.
//

#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <numeric>
#include <Eigen/Core>
#include <mutex>

#include "opencv2/core/core.hpp"

#include "Thirdparty/EDLines/LS.h"

// Edge Drawing header
#include "Thirdparty/EDTest/EdgeMap.h"
#include "Thirdparty/EDTest/EDLib.h"
#include "line3D.h"


namespace ORB_SLAM2 {
    class KeyFrame;
    class Map;
}

namespace cv {
    class Mat;
}

class Modeler;

class Cluster {
public:
    Cluster() {
        segments = cv::Mat::zeros(0, 6, CV_32F);
        centerSegment = cv::Mat::zeros(1, 6, CV_32F);
        index = 0;
    }
    unsigned long index;
    cv::Mat segments;
    cv::Mat centerSegment;
};


// 3D line segment detector
class LineDetector {
public:

    LineDetector();

    void Reset();

    // save time usage of methods in log file
    void Summary();
    // save line segments as obj files
    void SaveAllLineSegments();

    // run line3d++ on the keyframes (offline)
    void RunLine3Dpp(std::vector<ORB_SLAM2::KeyFrame*> vpKFs);
    // run decoupled line segment extraction (offline)
    void LineFittingEDLinesOffline(std::vector<ORB_SLAM2::KeyFrame*> vpKFs);
    // run edge-aided line segment extraction (offline)
    void LineFittingOffline(std::vector<ORB_SLAM2::KeyFrame*> vpKFs, Modeler* pModeler);

    // edge detection by EdgeDrawing
    void DetectEdgeMap(ORB_SLAM2::KeyFrame* kf);
    // 2D line segment detection by EDLines
    void DetectLineSegments(ORB_SLAM2::KeyFrame* kf);

    // 3D line segment extraction on a single keyframe
    void LineFitting(ORB_SLAM2::KeyFrame* kf);

    void ClosestPointOnLine(float a, float b, float c, int x, int y, float& cx, float& cy);
    int CountDepth(Pixel *pixelChain, int length, ORB_SLAM2::KeyFrame* kf);
    void LeastSquaresLineFit(Pixel* pixelChain, int initLength, float& u1, float& u2, float& u3, float& lineFitError);
    void LeastSquaresDepthFit(Pixel* pixelChain, int initLength, float la, float lb, float lc, float& u1, float& u2, float& depthFitError, ORB_SLAM2::KeyFrame* kf);
    float ComputePointDistance2Line(float a, float b, float c, Pixel pixel);
    float ComputePointDepth2Line(float a, float b, float c, float alpha, float beta, Pixel *pixelChain, Pixel pixel, ORB_SLAM2::KeyFrame* kf);
    void LineFit(Pixel *pixelChain, int noPixels, ORB_SLAM2::KeyFrame* kf);

    // incremental clustering line segments
    void MergeLines(ORB_SLAM2::KeyFrame* kf, Modeler* pModeler);

    bool CLoseEnough(Cluster c, cv::Mat l);
    void UpdateCluster(Cluster& c);
    void SaveClusteredSegments();

    std::string CurrentDateTime();
    std::string GetStringDateTime();

    std::vector<double> time_modeling;

private:

    std::string mStrDateTime;

    cv::Mat mAllLines;

    std::vector<Cluster> clusters;

    std::vector<ORB_SLAM2::KeyFrame *> vpKFs_global;

    std::vector<double> time_edge;
    std::vector<double> time_edlines;
    std::vector<double> time_fitting;
    std::vector<double> time_merging;
    std::vector<double> time_decoupled;
    std::vector<double> time_line3d;

};

#endif //LINEDETECTOR_H
