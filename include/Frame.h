/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* 
* Modification: EAO-SLAM
* Version: 1.0
* Created: 05/21/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include "YOLOv3SE.h"

// line
#include <line_lbd/line_descriptor.hpp>
#include <line_lbd/line_lbd_allclass.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// cube slam.
#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"

// // 3d cube.
// #include <object_slam/Object_landmark.h>
// #include <object_slam/g2o_Object.h>

typedef Eigen::Matrix<double,2,1> Vector2d;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class Object_2D;

// // for optimization, not used in this version.
// struct PairObjs
// {
//     Vector6d mvLineWithMappoint;     
//     Vector6d mvLineWithMapObjsCenter;
//     Vector4d mvLineWithFeature;      
//     int mWeightTwo;
// };

// // for optimization, not used in this version.
// struct SingleObj
// {
//     Vector3d mCenter3D;
//     Vector2d mCenter2D;
//     std::vector<Vector3d> mvSingleObjMapPointsPos;
//     std::vector<Vector2d> mvSingleObjMapfearurePos;
//     Vector6d mRect;                     
//     Vector6d mDetectedRect;             
//     int mWeightOne;                     
//     Vector3d mAssMapObjCenter;          
// };

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    Frame(  const cv::Mat &imGray, 
            const cv::Mat &rawImage,           
            const double &timeStamp, 
            ORBextractor* extractor,
            line_lbd_detect* line_lbd_ptr_frame,
            ORBVocabulary* voc, 
            cv::Mat &K, 
            cv::Mat &distCoef, 
            const float &bf, 
            const float &thDepth, 
            cv::Mat& grayimg, 
            cv::Mat& rgbimg);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // line.
    line_lbd_detect* mpline_lbd_ptr_frame;

    // Frame timestamp.
    double mTimeStamp;

    // NOTE [EAO-SLAM]
    cv::Mat mColorImage;
    cv::Mat mQuadricImage;
    bool finish_detected;           // whether finished object detection.
    std::vector<BoxSE> boxes;       // object box, vector<BoxSE> format.
    Eigen::MatrixXd boxes_eigen;    // object box, Eigen::MatrixXd format.
    bool have_detected;             // whether detected objects in current frame.

    // // for optimization.
    // typedef Eigen::Matrix<double,5,1> Vector5d;
    // typedef Eigen::Matrix<double,6,1> Vector6d;
    // std::vector<Vector6d> LineFrameToFrameWithMappoint;
    // std::vector<Vector5d> LineFrameToFrameWithFeature;
    // std::vector<PairObjs> mvPairObjs;    
    // std::vector<SingleObj> mvSingleObj;
    
    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // pass img to keyframe, using to semidense create
   cv::Mat im_;
   cv::Mat rgb_;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // note [EAO-SLAM].
    std::vector<Object_2D*> mvObjectFrame;          // 2d object in current frame.
    std::vector<Object_2D*> mvLastObjectFrame;      // last frame.
    std::vector<Object_2D*> mvLastLastObjectFrame;  // last last frame.
    bool AppearNewObject = false;                   // Whether new objects appear in the current frame.

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // line.
    std::vector< KeyLine> keylines_raw, keylines_out;
    cv::Mat all_lines_mat;
    Eigen::MatrixXd all_lines_eigen;
    std::vector<Eigen::MatrixXd> vObjsLines;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;
    cv::Mat mGroundtruthPose_mat;           // camera groundtruth.
    Eigen::Matrix4d mGroundtruthPose_eigen; // camera groundtruth.

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
