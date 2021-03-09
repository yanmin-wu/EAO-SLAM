/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* 
* Modification: EAO-SLAM
* Version: 1.0
* Created: 05/16/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>

// line.
#include <line_lbd/line_descriptor.hpp>
#include <line_lbd/line_lbd_allclass.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();

    // [EAO-SLAM]
    cv::Mat DrawYoloFrame();
    cv::Mat GetRawColorImage();
    cv::Mat GetQuadricImage();

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    cv::Mat DrawYoloInfo(cv::Mat &im, bool bText);

	std::vector<cv::Scalar> colors = {  cv::Scalar(0,0,255),      
                                        cv::Scalar(0,255,0),      
                                        cv::Scalar(255,0,0),      
                                        cv::Scalar(0,255,255)
                                        };

    std::vector<std::string> class_names = { 
                                        "person",
                                        "bicycle",
                                        "car",
                                        "motorbike",
                                        "aeroplane",
                                        "bus",
                                        "train",
                                        "truck",
                                        "boat",
                                        "traffic light",
                                        "fire hydrant",
                                        "stop sign",
                                        "parking meter",
                                        "bench",
                                        "bird",
                                        "cat",
                                        "dog",
                                        "horse",
                                        "sheep",
                                        "cow",
                                        "elephant",
                                        "bear",
                                        "zebra",
                                        "giraffe",
                                        "backpack",
                                        "umbrella",
                                        "handbag",
                                        "tie",
                                        "suitcase",
                                        "frisbee",
                                        "skis",
                                        "snowboard",
                                        "sports ball",
                                        "kite",
                                        "baseball bat",
                                        "baseball glove",
                                        "skateboard",
                                        "surfboard",
                                        "tennis racket",
                                        "bottle",
                                        "wine glass",
                                        "cup",
                                        "fork",
                                        "knife",
                                        "spoon",
                                        "bowl",
                                        "banana",
                                        "apple",
                                        "sandwich",
                                        "orange",
                                        "broccoli",
                                        "carrot",
                                        "hot dog",
                                        "pizza",
                                        "donut",
                                        "cake",
                                        "chair",
                                        "sofa",
                                        "pottedplant",
                                        "bed",
                                        "diningtable",
                                        "toilet",
                                        "tvmonitor",
                                        "laptop",
                                        "mouse",
                                        "remote",
                                        "keyboard",
                                        "cell phone",
                                        "microwave",
                                        "oven",
                                        "toaster",
                                        "sink",
                                        "refrigerator",
                                        "book",
                                        "clock",
                                        "vase",
                                        "scissors",
                                        "teddy bear",
                                        "hair drier",
                                        "toothbrush"
                                        };

    // Info of the frame to be drawn
    cv::Mat mIm;
    cv::Mat mRGBIm; 
    cv::Mat mQuadricIm; 

    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    // lines.
    std::vector< KeyLine> Dkeylines_raw, Dkeylines_out;
    double DTimeStamp;

    // bounding box.
    std::vector<BoxSE> Dboxes;
    bool have_detected;
    std::vector<Eigen::MatrixXd> DObjsLines;    // object lines.

    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
