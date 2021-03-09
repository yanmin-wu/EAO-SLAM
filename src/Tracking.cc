/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* 
* Modification: EAO-SLAM
* Version: 1.0
* Created: 03/21/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>

#include <mutex>

#include "ProbabilityMapping.h"

// cube slam
#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"

// cube slam
// #include <object_slam/Object_landmark.h>
// #include <object_slam/g2o_Object.h>

#include "Object.h"
#include "FrameDrawer.h"

#include <cmath>
#include<algorithm>

using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float,5,1> Vector5f;

namespace ORB_SLAM2
{

bool Tracking::mbReadedGroundtruth = false;
int frame_id_tracking = -1;

// rank.
// typedef Eigen::Vector4f VI;
typedef Vector5f VI;
int index=0;
bool VIC(const VI& lhs, const VI& rhs) 
{
    return lhs[index] > rhs[index];
}

Tracking::Tracking( System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB, 
                    const string &strSettingPath, const int sensor, const string &flag) : 
                    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
                                                                                                                                                                                              mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys),
                                                                                                                                                                                              mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl
         << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if (DistCoef.rows == 5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    // demo flag.
    mflag = flag;

    // STEP line detect +++++++++++++++++++++++++++++++++++++++++++
    bool use_LSD_algorithm = false; 
    bool save_to_imgs = false;
    bool save_to_txts = false;

    // initial a line detector.
    int numOfOctave_ = 1;
    float Octave_ratio = 2.0;
    line_lbd_ptr = new line_lbd_detect(numOfOctave_, Octave_ratio);

    line_lbd_ptr->use_LSD = use_LSD_algorithm;
    line_lbd_ptr->save_imgs = save_to_imgs;
    line_lbd_ptr->save_txts = save_to_txts;
    // line detect ------------------------------------------------

    // the threshold of removing short line.
    line_lbd_ptr->line_length_thres = 15;

    cout << endl
         << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if (sensor == System::STEREO || sensor == System::RGBD)
    {
        mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
        cout << endl
             << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if (sensor == System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if (mDepthMapFactor == 0)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    // STEP read groundtruth ++++++++++++++++++++++++++++++++++++++++++++++++
    // notice: Only the camera pose of the initial frame is used to determine the ground plane normal.
    if (mbReadedGroundtruth == false)
    {
        ifstream infile("./data/groundtruth.txt", ios::in);
        if (!infile.is_open())
        {
            cout << "tum groundtruth file open fail" << endl;
            exit(233);
        }
        else
        {
            std::cout << "read groundtruth.txt" << std::endl;
            mbReadedGroundtruth = true;
        }

        vector<double> row;
        double tmp;
        string line;
        cv::Mat cam_pose_mat;

        string s0;
        getline(infile, s0);
        getline(infile, s0);
        getline(infile, s0);

        // save as vector<vector<int>> _mat format.
        while (getline(infile, line))
        {
            // string to int.
            istringstream istr(line);
            while (istr >> tmp)
            {
                row.push_back(tmp);
            }

            mGroundtruth_mat.push_back(row); // vector<int> row.

            row.clear();
            istr.clear();
            line.clear();
        }
        infile.close();
    }
    // read groundtruth --------------------------------------------------
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void Tracking::SetSemiDenseMapping(ProbabilityMapping *pSemiDenseMapping)
{
    mpSemiDenseMapping = pSemiDenseMapping;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
        {
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
        }
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
        {
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    if (mDepthMapFactor != 1 || imDepth.type() != CV_32F)
        ;
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im,
                                     const double &timestamp,
                                     const bool bSemanticOnline)
{
    frame_id_tracking++;

    cv::Mat rawImage = im.clone();

    if (bSemanticOnline)
    {
        // TODO send image to semantic thread.
    }

    mImGray = im;

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    // undistort image
    cv::Mat gray_imu;
    cv::undistort(mImGray, gray_imu, mK, mDistCoef);

    cv::Mat rgb_imu;
    cv::undistort(im, rgb_imu, mK, mDistCoef);
    if (rgb_imu.channels() == 1)
    {
        cvtColor(rgb_imu, rgb_imu, CV_GRAY2RGB);
    }
    else if (rgb_imu.channels() == 3)
    {
        if (!mbRGB)
            cvtColor(rgb_imu, rgb_imu, CV_BGR2RGB);
    }
    else if (rgb_imu.channels() == 4)
    {
        if (mbRGB)
            cvtColor(rgb_imu, rgb_imu, CV_RGBA2RGB);
        else
            cvtColor(rgb_imu, rgb_imu, CV_BGRA2RGB);
    }

    // STEP 1. Construct Frame.
    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
        mCurrentFrame = Frame(rawImage,         // color image.
                              mImGray,
                              timestamp,
                              mpIniORBextractor,
                              line_lbd_ptr,     // [EAO] line extractor.
                              mpORBVocabulary,
                              mK,
                              mDistCoef,
                              mbf,
                              mThDepth,
                              gray_imu,
                              rgb_imu);
    else
        mCurrentFrame = Frame(rawImage,         // color image.
                              mImGray,
                              timestamp,
                              mpORBextractorLeft,
                              line_lbd_ptr,     // [EAO] line extractor.
                              mpORBVocabulary,
                              mK,
                              mDistCoef,
                              mbf,
                              mThDepth,
                              gray_imu,
                              rgb_imu);

    // STEP object detection.+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    vector<vector<int>> _mat;
    mCurrentFrame.have_detected = false;
    mCurrentFrame.finish_detected = false;
    if (bSemanticOnline)
    {
        // TODO online detect.
    }
    else
    {
        // offline object box.
        ifstream infile("./data/yolo_txts/" + to_string(timestamp) + ".txt", ios::in);
        if (!infile.is_open())
        {
            cout << "yolo_detection file open fail" << endl;
            exit(233);
        }
        // else
        //     cout << "read offline boundingbox" << endl;

        vector<int> row; // one row, one object.
        int tmp;
        string line;

        // save as vector<vector<int>> format.
        while (getline(infile, line))
        {
            // string to int.
            istringstream istr(line);
            while (istr >> tmp)
            {
                row.push_back(tmp);
            }

            _mat.push_back(row); // vector<vector<int>>.
            row.clear();
            istr.clear();
            line.clear();
        }
        infile.close();

        //  vector<vector<int>> --> std::vector<BoxSE>.
        std::vector<BoxSE> boxes_offline;
        for (auto &mat_row : _mat)
        {
            BoxSE box;
            box.m_class = mat_row[0];
            box.m_score = mat_row[5];
            box.x = mat_row[1];
            box.y = mat_row[2];
            box.width = mat_row[3];
            box.height = mat_row[4];
            // box.m_class_name = "";
            boxes_offline.push_back(box);
        }
        std::sort(boxes_offline.begin(), boxes_offline.end(), [](BoxSE a, BoxSE b) -> bool {
            return a.m_score > b.m_score;
        });
        // save to current frame.
        mCurrentFrame.boxes = boxes_offline;

        // std::vector<BoxSE> --> Eigen::MatrixXd.
        int i = 0;
        Eigen::MatrixXd eigenMat;
        eigenMat.resize((int)mCurrentFrame.boxes.size(), 5);
        for (auto &box : mCurrentFrame.boxes)
        {
            // std::cout << box.m_class << " " << box.x << " " << box.y << " "
            //           << box.width << " " << box.height << " " << box.m_score << std::endl;
            /**
            * keyboard  66 199 257 193 51 
            * mouse     64 377 320 31 39 
            * cup       41 442 293 51 63 
            * tvmonitor 62 232 93 156 141 
            * remote    65 44 260 38 57
            */
            eigenMat(i, 0) = box.x;
            eigenMat(i, 1) = box.y;
            eigenMat(i, 2) = box.width;
            eigenMat(i, 3) = box.height;
            eigenMat(i, 4) = box.m_score;
            i++;
        }
        // save to current frame.
        mCurrentFrame.boxes_eigen = eigenMat;
    }
    // there are objects in current frame?
    if (!mCurrentFrame.boxes.empty())
        mCurrentFrame.have_detected = true;
    // object detection.------------------------------------------------------------------------

    // STEP get current camera groundtruth by timestamp. +++++++++++++++++++++++++++++++++++++++++++
    // notice: only use the first frame's pose.
    string timestamp_string = to_string(timestamp);
    string timestamp_short_string = timestamp_string.substr(0, timestamp_string.length() - 4);

    Eigen::MatrixXd truth_frame_poses(1, 8); // camera pose Eigen format.
    cv::Mat cam_pose_mat;                    // camera pose Mat format.

    for (auto &row : mGroundtruth_mat)
    {
        string row_string = to_string(row[0]);
        string row_short_string = row_string.substr(0, row_string.length() - 4);

        if (row_short_string == timestamp_short_string)
        {
            // vector --> Eigen.
            for (int i = 0; i < (int)row.size(); i++)
            {
                truth_frame_poses(0) = row[0];
                truth_frame_poses(1) = row[1];
                truth_frame_poses(2) = row[2];
                truth_frame_poses(3) = row[3];
                truth_frame_poses(4) = row[4];
                truth_frame_poses(5) = row[5];
                truth_frame_poses(6) = row[6];
                truth_frame_poses(7) = row[7];
            }

            // Eigen --> SE3.
            g2o::SE3Quat cam_pose_se3(truth_frame_poses.row(0).tail<7>());
            // std::cout << "cam_pose_se3\n" << cam_pose_se3 << std::endl;

            // SE3 --> Mat.
            cam_pose_mat = Converter::toCvMat(cam_pose_se3);

            // save to current frame.
            mCurrentFrame.mGroundtruthPose_mat = cam_pose_mat;
            if (!mCurrentFrame.mGroundtruthPose_mat.empty())
            {
                mCurrentFrame.mGroundtruthPose_eigen = Converter::toEigenMatrixXd(mCurrentFrame.mGroundtruthPose_mat);
            }
            break;
        }
        else
        {
            mCurrentFrame.mGroundtruthPose_mat = cv::Mat::zeros(4, 4, CV_8UC1);
            mCurrentFrame.mGroundtruthPose_eigen = Eigen::Matrix4d::Zero(4, 4);
        }
    }
    // get the camera groundtruth by timestamp. ----------------------------------------------------------------------

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if (mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if (mState == NOT_INITIALIZED)
    {
        if (mSensor == System::STEREO || mSensor == System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();
        mpFrameDrawer->Update(this);

        if (mState != OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if (!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if (mState == OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    // note [EAO] for this opensource version, the mainly modifications are in the TrackWithMotionModel().
                    bOK = TrackWithMotionModel();
                    if (!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Only Tracking: Local Mapping is deactivated

            if (mState == LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if (!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if (!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO)
                        {
                            for (int i = 0; i < mCurrentFrame.N; i++)
                            {
                                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (!mbOnlyTracking)
        {
            if (bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if (bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if (bOK)
            mState = OK;
        else
            mState = LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if (bOK)
        {
            // Update motion model
            if (!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean temporal point matches
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
            {
                MapPoint *pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if (NeedNewKeyFrame() == 1)
                CreateNewKeyFrame(false);
            else if (NeedNewKeyFrame() == 2)    // note [EAO] create keyframes by the new object.
            {
                CreateNewKeyFrame(true);
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST)
        {
            if (mpMap->KeyFramesInMap() <= 5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if (!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState == LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState == LOST);
    }
}

void Tracking::StereoInitialization()
{
    if (mCurrentFrame.N > 500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

        // Create KeyFrame
        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                pNewMP->AddObservation(pKFini, i);
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i] = pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState = OK;
    }
}

void Tracking::MonocularInitialization()
{
    if (!mpInitializer)
    {
        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > 100)
        {
            mInitialFrame = Frame(mCurrentFrame);

            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            if (mpInitializer)
                delete mpInitializer;

            mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if ((int)mCurrentFrame.mvKeys.size() <= 100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,
                                                       mCurrentFrame,
                                                       mvbPrevMatched,
                                                       mvIniMatches,
                                                       100);

        // Check if there are enough correspondences
        if (nmatches < 100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            return;
        }

        cv::Mat Rcw;                 // Current Camera Rotation
        cv::Mat tcw;                 // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
            {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(Tcw.rowRange(0, 3).col(3));
            mCurrentFrame.SetPose(Tcw);

            mInitialSecendFrame = Frame(mCurrentFrame); // [EAO] the second frame when initialization.

            CreateInitialMapMonocular();
        }
    }
}

// note [EAO] modify: rotate the world coordinate to the initial frame.
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++)
    {
        if (mvIniMatches[i] < 0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    // NOTE [EAO] rotate the world coordinate to the initial frame (groundtruth provides the normal vector of the ground).
    // only use the groundtruth of the first frame.
    cv::Mat InitToGround = mInitialFrame.mGroundtruthPose_mat;

    cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = InitToGround.rowRange(0, 3).col(3);
    cv::Mat Rinv = R.t();
    cv::Mat Ow = -Rinv * t;
    cv::Mat GroundToInit = cv::Mat::eye(4, 4, CV_32F);
    Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
    Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));

    bool build_worldframe_on_ground = true;
    if (build_worldframe_on_ground) // transform initial pose and map to ground frame
    {
        pKFini->SetPose(pKFini->GetPose() * GroundToInit);
        pKFcur->SetPose(pKFcur->GetPose() * GroundToInit);

        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(InitToGround.rowRange(0, 3).colRange(0, 3) * pMP->GetWorldPos() + InitToGround.rowRange(0, 3).col(3));
            }
        }
    }
    // [EAO] rotate the world coordinate to the initial frame -----------------------------------------------------------

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for (int i = 0; i < mLastFrame.N; i++)
    {
        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if (pMP)
        {
            MapPoint *pRep = pMP->GetReplaced();
            if (pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if (nmatches < 15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for (int i = 0; i < mLastFrame.N; i++)
    {
        float z = mLastFrame.mvDepth[i];
        if (z > 0)
        {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1)
        {
            bCreateNew = true;
        }

        if (bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

    // ******************************
    //      STEP 0. Cube SLAM       *
    // ******************************
    bool bCubeslam = false;
    if((mCurrentFrame.mnId > 10) && bCubeslam)
    {
        Eigen::Matrix3d calib; 
        calib << 535.4,  0,  320.1,
                 0,  539.2, 247.6,
                 0,      0,     1;

        detect_3d_cuboid detect_cuboid_obj;
        detect_cuboid_obj.whether_plot_detail_images = false;	
        detect_cuboid_obj.whether_plot_final_images = false;	
        detect_cuboid_obj.print_details = false;  				
        detect_cuboid_obj.set_calibration(calib);				
        detect_cuboid_obj.whether_sample_bbox_height = false;	
        detect_cuboid_obj.whether_sample_cam_roll_pitch = false; 
        detect_cuboid_obj.nominal_skew_ratio = 2;				
        detect_cuboid_obj.whether_save_final_images = true;

        std::vector<ObjectSet> frames_cuboids;
        
        detect_cuboid_obj.detect_cuboid(mCurrentFrame.mColorImage,
                                        mCurrentFrame.mGroundtruthPose_eigen,
                                        mCurrentFrame.boxes_eigen,
                                        mCurrentFrame.all_lines_eigen, 
                                        frames_cuboids);
        
        cv::imwrite("cubeslam_results/" + to_string(mCurrentFrame.mnId) + ".png", detect_cuboid_obj.cuboids_2d_img);
    }
    // Cube SLAM END ---------------------------------------------------------

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

    // *****************************
    // STEP 1. construct 2D object *
    // *****************************
    vector<Object_2D *> objs_2d;
    cv::Mat image = mCurrentFrame.mColorImage.clone();
    for (auto &box : mCurrentFrame.boxes)
    {
        Object_2D *obj = new Object_2D;

        // copy object bounding box and initialize the 3D center.
        obj->CopyBoxes(box);
        obj->sum_pos_3d = cv::Mat::zeros(3, 1, CV_32F);

        objs_2d.push_back(obj);
    }
    // construct 2D object END ---------------

    // Project points seen in previous frame
    int th;
    if (mSensor != System::STEREO)
        th = 15;
    else
        th = 7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches < 20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
    }

    if (nmatches < 20)
        return false;

    // ***************************************
    // STEP 2. associate objects with points *
    // ***************************************
    AssociateObjAndPoints(objs_2d);

    // ***************************************
    // STEP 3. associate objects with lines *
    // ***************************************
    AssociateObjAndLines(objs_2d);

    // ***************************************************
    // STEP 4. compute the mean and standard of points.*
    // Erase outliers (camera frame) by boxplot.*
    // **************************************************
    for (auto &obj : objs_2d)
    {
        // compute the mean and standard.
        obj->ComputeMeanAndStandardFrame();

        // If the object has too few points, ignore.
        if (obj->Obj_c_MapPonits.size() < 8)
            continue;

        // Erase outliers by boxplot.
        obj->RemoveOutliersByBoxPlot(mCurrentFrame);
    }
    // Erase outliers of obj_2d END ----------------------

    // **************************************************************************
    // STEP 5. construct the bounding box by object feature points in the image.*
    // **************************************************************************
    // bounding box detected by yolo |  bounding box constructed by object points.
    //  _______________                 //  _____________
    // |   *        *  |                // |   *        *|
    // |    *  *       |                // |    *  *     |
    // |*      *  *    |                // |*      *  *  |
    // | *   *    *    |                // | *   *    *  |
    // |   *       *   |                // |___*_______*_|
    // |_______________|
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);
    for (auto &obj : objs_2d)
    {
        // object 3D center (world).
        obj->_Pos = obj->sum_pos_3d / obj->Obj_c_MapPonits.size();
        obj->mCountMappoint = obj->Obj_c_MapPonits.size(); // point number.
        // world -> camera.
        cv::Mat x3Dc = Rcw * obj->_Pos + tcw;
        const float xc = x3Dc.at<float>(0);
        const float yc = x3Dc.at<float>(1);
        const float invzc = 1.0 / x3Dc.at<float>(2);
        // camera -> image.
        float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
        float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;
        obj->point_center_2d = cv::Point2f(u, v);    // 3D center project to image. no use in this opensource version.

        // record the coordinates of each point in the xy(uv) directions.
        vector<float> x_pt;
        vector<float> y_pt;
        for (auto &pMP : obj->Obj_c_MapPonits)
        {
            float u = pMP->feature.pt.x;
            float v = pMP->feature.pt.y;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        if (x_pt.size() < 4) // ignore.
            continue;

        // extremum in xy(uv) direction
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        // make insure in the image.
        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > image.cols)
            x_max = image.cols;
        if (y_max > image.rows)
            y_max = image.rows;

        // the bounding box constructed by object feature points.
        obj->mRectFeaturePoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    }
    // construct bounding box by feature points END -----------------------------------

    // **********************************************************************************************
    // STEP 6. remove 2d bad bounding boxes.
    // Due to the complex scene and Yolo error detection, some poor quality objects need to be removed.
    // The strategy can be adjusted and is not unique, such as:
    // 1. objects overlap with too many object;
    // 2. objects with too few points;
    // 3. objects with too few points and on the edge of the image;
    // 4. objects too large and take up more than half of the image;
    // TODO and so on ......
    // **********************************************************************************************
    // overlap with too many objects.
    for (size_t f = 0; f < objs_2d.size(); ++f)
    {
        int num = 0;
        for (size_t l = 0; l < objs_2d.size(); ++l)
        {
            if (f == l)
                continue;

            if (Converter::bboxOverlapratioLatter(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.05)
                num++;
        }
        // overlap with more than 3 objects.
        if (num > 4)
            objs_2d[f]->bad = true;
    }
    for (size_t f = 0; f < objs_2d.size(); ++f)
    {
        if (objs_2d[f]->bad)
            continue;

        // ignore the error detect by yolo.
        if ((objs_2d[f]->_class_id == 0) || (objs_2d[f]->_class_id == 63) || (objs_2d[f]->_class_id == 15))
            objs_2d[f]->bad = true;

        // too large in the image.
        if ((float)objs_2d[f]->mBoxRect.area() / (float)(image.cols * image.rows) > 0.5)
            objs_2d[f]->bad = true;

        // too few object points.
        if (objs_2d[f]->Obj_c_MapPonits.size() < 5)
            objs_2d[f]->bad = true;

        // object points too few and the object on the edge of the image.
        else if ((objs_2d[f]->Obj_c_MapPonits.size() >= 5) && (objs_2d[f]->Obj_c_MapPonits.size() < 10))
        {
            if ((objs_2d[f]->mBox.x < 20) || (objs_2d[f]->mBox.y < 20) ||
                (objs_2d[f]->mBox.x + objs_2d[f]->mBox.width > image.cols - 20) ||
                (objs_2d[f]->mBox.y + objs_2d[f]->mBox.height > image.rows - 20))
            {
                objs_2d[f]->bad = true;
            }
        }

        // mark the object that on the edge of the image.
        if (((objs_2d[f]->mBox.x < 5) || (objs_2d[f]->mBox.y < 5) ||
            (objs_2d[f]->mBox.x + objs_2d[f]->mBox.width > image.cols - 5) ||
            (objs_2d[f]->mBox.y + objs_2d[f]->mBox.height > image.rows - 5)))
        {
            objs_2d[f]->bOnEdge = true;
        }

        // when the overlap is large, only one object remains.
        for (size_t l = 0; l < objs_2d.size(); ++l)
        {
            if (objs_2d[l]->bad)
                continue;

            if (f == l)
                continue;

            // retain objects which with high probability.
            if (Converter::bboxOverlapratio(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.3)
            {
                if (objs_2d[f]->mScore < objs_2d[l]->mScore)
                    objs_2d[f]->bad = true;
                else if (objs_2d[f]->mScore >= objs_2d[l]->mScore)
                    objs_2d[l]->bad = true;
            }
            // if one object surrounds another, keep the larger one.
            if (Converter::bboxOverlapratio(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.05)
            {
                if (Converter::bboxOverlapratioFormer(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.85)
                    objs_2d[f]->bad = true;
                if (Converter::bboxOverlapratioLatter(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.85)
                    objs_2d[l]->bad = true;
            }
        }
    }
    // erase the bad object.
    vector<Object_2D *>::iterator it;
    for (it = objs_2d.begin(); it != objs_2d.end(); )
    {
        if ((*it)->bad == true)
            it = objs_2d.erase(it); // erase.
        else
        {
            // if ((*it)->Obj_c_MapPonits.size() >= 5)
            // {
            //     cv::rectangle(image,                   
            //                     (*it)->mBoxRect,         
            //                     cv::Scalar(100, 100, 256), 
            //                     2);                      
            // }

            // cv::putText(image, to_string((*it)->_class_id),
            //             (*it)->box_center_2d,
            //             cv::FONT_HERSHEY_SIMPLEX, 0.5,
            //             cv::Scalar(0, 255, 255), 2);

            // std::string imname_rect = "./box/" + to_string(mCurrentFrame.mTimeStamp) + ".jpg";
            // cv::imwrite(imname_rect, image);

            ++it;
        }
    }
    // remove 2d bad bounding boxes END ------------------------------------------------------

    // *************************************************************
    // STEP 7. copy objects in the last frame after initialization.*
    // *************************************************************
    if ((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        // copy objects in the last frame.
        mCurrentFrame.mvLastObjectFrame = mLastFrame.mvObjectFrame;

        // copy objects in the penultimate frame.
        if (!mLastFrame.mvLastObjectFrame.empty())
            mCurrentFrame.mvLastLastObjectFrame = mLastFrame.mvLastObjectFrame;
    }
    // copy objects END -------------------------------------------------------

    // *******************************************************************************
    // STEP 8. Merges objects with 5-10 points  between two adjacent frames.
    // Advantage: Small objects with too few points, can be merged to keep them from being eliminated.
    // (The effect is not very significant.)
    // *******************************************************************************
    bool bMergeTwoObj = true;
    if ((!mCurrentFrame.mvLastObjectFrame.empty()) && bMergeTwoObj)
    {
        // object in current frame.
        for (size_t k = 0; k < objs_2d.size(); ++k)
        {
            // ignore objects with more than 10 points.
            if (objs_2d[k]->Obj_c_MapPonits.size() >= 10)
                continue;

            // object in last frame.
            for (size_t l = 0; l < mCurrentFrame.mvLastObjectFrame.size(); ++l)
            {
                // ignore objects with more than 10 points.
                if (mCurrentFrame.mvLastObjectFrame[l]->Obj_c_MapPonits.size() >= 10)
                    continue;

                // merge two objects.
                if (Converter::bboxOverlapratio(objs_2d[k]->mBoxRect, mCurrentFrame.mvLastObjectFrame[l]->mBoxRect) > 0.5)
                {
                    objs_2d[k]->MergeTwoFrameObj(mCurrentFrame.mvLastObjectFrame[l]);
                    break;
                }
            }
        }
    }
    // merge objects in two frame END -----------------------------------------------

    // ************************************
    // STEP 9. Initialize the object map  *
    // ************************************
    if ((mCurrentFrame.mnId > mInitialSecendFrame.mnId) && mbObjectIni == false)
        InitObjMap(objs_2d);

    // **************************************************************
    // STEP 10. Data association after initializing the object map. *
    // **************************************************************
    if ((mCurrentFrame.mnId > mnObjectIniFrameID) && (mbObjectIni == true))
    {
        // step 10.1 points of the object that appeared in the last 30 frames 
        // are projected into the image to form a projection bounding box.
        for (int i = 0; i < (int)mpMap->mvObjectMap.size(); i++)
        {
            if (mpMap->mvObjectMap[i]->bBadErase)
                continue;

            // object appeared in the last 30 frames.
            if (mpMap->mvObjectMap[i]->mnLastAddID > mCurrentFrame.mnId - 30)
                mpMap->mvObjectMap[i]->ComputeProjectRectFrame(image, mCurrentFrame);
            else
            {
                mpMap->mvObjectMap[i]->mRectProject = cv::Rect(0, 0, 0, 0);
            }
        }

        // step 10.2 data association.
        for (size_t k = 0; k < objs_2d.size(); ++k)
        {
            // ignore object with less than 5 points.
            if (objs_2d[k]->Obj_c_MapPonits.size() < 5)
            {
                objs_2d[k]->few_mappoint = true;
                objs_2d[k]->current = false;
                continue;
            }

            // note: data association.
            objs_2d[k]->ObjectDataAssociation(mpMap, mCurrentFrame, image, mflag);
        }

        // step 10.3 remove objects with too few observations.
        for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
        {
            if(mflag == "NA")
                continue;

            if (mpMap->mvObjectMap[i]->bBadErase)
                continue;

            int df = (int)mpMap->mvObjectMap[i]->mObjectFrame.size();
            if (df < 10)
            {
                // not been observed in the last 30 frames.
                if (mpMap->mvObjectMap[i]->mnLastAddID < (mCurrentFrame.mnId - 30))
                {
                    if (df < 5)
                        mpMap->mvObjectMap[i]->bBadErase = true;

                    // if not overlap with other objects, don't remove.
                    else
                    {
                        bool overlap = false;
                        for (int j = (int)mpMap->mvObjectMap.size() - 1; j >= 0; j--)
                        {
                            if (mpMap->mvObjectMap[j]->bBadErase || (i == j))
                                continue;

                            if (mpMap->mvObjectMap[i]->WhetherOverlap(mpMap->mvObjectMap[j]))
                            {
                                overlap = true;
                                break;
                            }
                        }
                        if (overlap)
                            mpMap->mvObjectMap[i]->bBadErase = true;
                    }
                }
            }
        }

        // step 10.4 Update the co-view relationship between objects. (appears in the same frame).
        for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
        {
            if (mpMap->mvObjectMap[i]->mnLastAddID == mCurrentFrame.mnId)
            {
                for (int j = (int)mpMap->mvObjectMap.size() - 1; j >= 0; j--)
                {
                    if (i == j)
                        continue;

                    if (mpMap->mvObjectMap[j]->mnLastAddID == mCurrentFrame.mnId)
                    {
                        int nObjId = mpMap->mvObjectMap[j]->mnId;

                        map<int, int>::iterator sit;
                        sit = mpMap->mvObjectMap[i]->mmAppearSametime.find(nObjId);

                        if (sit != mpMap->mvObjectMap[i]->mmAppearSametime.end())
                        {
                            int sit_sec = sit->second;
                            mpMap->mvObjectMap[i]->mmAppearSametime.erase(nObjId);
                            mpMap->mvObjectMap[i]->mmAppearSametime.insert(make_pair(nObjId, sit_sec + 1));
                        }
                        else
                            mpMap->mvObjectMap[i]->mmAppearSametime.insert(make_pair(nObjId, 1));   // first co-view.
                    }
                }
            }
        }

        // step 10.5 Merge potential associate objects (see mapping thread).

        // step 10.6 Estimate the orientation of objects.
        for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
        {
            // map object.
            Object_Map* objMap = mpMap->mvObjectMap[i];

            if (objMap->bBadErase)
                continue;

            if (objMap->mnLastAddID < mCurrentFrame.mnId - 5)
                continue;

            // estimate only regular objects.
            if (((objMap->mnClass == 73) || (objMap->mnClass == 64) || (objMap->mnClass == 65) 
                || (objMap->mnClass == 66) || (objMap->mnClass == 56)))
            {
                // objects appear in current frame.
                if(objMap->mnLastAddID == mCurrentFrame.mnId)
                {
                    SampleObjYaw(objMap);   // note: sample object yaw.
                }
            }

            // step 10.7 project quadrics to the image (only for visualization).
            cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
            axe.at<float>(0) = mpMap->mvObjectMap[i]->mCuboid3D.lenth / 2;
            axe.at<float>(1) = mpMap->mvObjectMap[i]->mCuboid3D.width / 2;  
            axe.at<float>(2) = mpMap->mvObjectMap[i]->mCuboid3D.height / 2;

            // object pose (world).
            cv::Mat Twq = Converter::toCvMat(mpMap->mvObjectMap[i]->mCuboid3D.pose);

            // Projection Matrix K[R|t].
            cv::Mat P(3, 4, CV_32F);
            Rcw.copyTo(P.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(P.rowRange(0, 3).col(3));
            P = mCurrentFrame.mK * P;

            // draw.
            image = DrawQuadricProject( this->mCurrentFrame.mQuadricImage,
                                        P,   
                                        axe, 
                                        Twq, 
                                        mpMap->mvObjectMap[i]->mnClass);
        }
    } // data association END ----------------------------------------------------------------

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mbOnlyTracking)
    {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking)
                {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        return false;

    if (mnMatchesInliers < 30)
        return false;
    else
        return true;
}

// note [EAO] Modify: Create keyframes by new object.
int Tracking::NeedNewKeyFrame()
{
    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0;
    int nTotal = 0;
    if (mSensor != System::MONOCULAR)
    {
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
            {
                nTotal++;
                if (mCurrentFrame.mvpMapPoints[i])
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        nMap++;
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap = 1;
        nTotal = 1;
    }

    const float ratioMap = (float)nMap / fmax(1.0f, nTotal);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    if (mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    float thMapRatio = 0.35f;
    if (mnMatchesInliers > 300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || ratioMap < 0.3f);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || ratioMap < thMapRatio) && mnMatchesInliers > 15);

    // note [EAO] create new keyframe by object.
    bool c1d = false;
    if (mCurrentFrame.AppearNewObject)
        c1d = true;

    if ((c1a || c1b || c1c) && c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle)
        {
            return 1;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return 1;
                else
                    return 0;
            }
            else
                return 0;
        }
    }
    // note [EAO] create new keyframe by object.
    else if (c1d)
    {
        if (bLocalMappingIdle)
        {
            return 2;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return 2;
                else
                    return 0;
            }
            else
                return 0;
        }
    }
    else
        return 0;
}

void Tracking::CreateNewKeyFrame(bool CreateByObjs)
{
    if (!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // save ovjects to keyframe.
    pKF->objects_kf = mCurrentFrame.mvObjectFrame;

    // keyframe created by objects.
    if (CreateByObjs)
        pKF->mbCreatedByObjs = true;

    if (mSensor != System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }

                if (bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                    pNewMP->AddObservation(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP)
        {
            if (pMP->isBad())
            {
                *vit = static_cast<MapPoint *>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if (nToMatch > 0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if (mSensor == System::RGBD)
            th = 3;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
    {
        KeyFrame *pKF = *itKF;
        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
        {
            MapPoint *pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP->isBad())
            {
                const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }
    }

    if (keyframeCounter.empty())
        return;

    int max = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
    {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max)
        {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)
            break;

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
        {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->GetChilds();
        for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
        {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad())
            {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->GetParent();
        if (pParent)
        {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                break;
            }
        }
    }

    if (pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++)
    {
        KeyFrame *pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch)
    {
        for (int i = 0; i < nKFs; i++)
        {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver *pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore)
            {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++)
                {
                    if (vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame.N; io++)
                    if (mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50)
                {
                    int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50)
                        {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // Final optimization
                            if (nGood + nadditional >= 50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for (int io = 0; io < mCurrentFrame.N; io++)
                                    if (mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }
}

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while (!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Reset Semi Dense Mapping
    cout << "Reseting Semi Dense Mapping...";
    mpSemiDenseMapping->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    KeyFrame::nNextMappingId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer *>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

// BRIEF [EAO] associate objects with points.
void Tracking::AssociateObjAndPoints(vector<Object_2D *> objs_2d)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

            if (!pMP->isBad())
            {
                for (size_t k = 0; k < objs_2d.size(); ++k)
                {
                    if (objs_2d[k]->mBoxRect.contains(mCurrentFrame.mvKeysUn[i].pt))// in rect.
                    {
                        cv::Mat PointPosWorld = pMP->GetWorldPos();                 // world frame.
                        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;         // camera frame.

                        pMP->object_view = true;                  // the point is associated with an object.
                        pMP->frame_id.insert(mCurrentFrame.mnId); // no use.
                        pMP->feature = mCurrentFrame.mvKeysUn[i]; // coordinate in current frame.

                        // object points.
                        objs_2d[k]->Obj_c_MapPonits.push_back(pMP);

                        // summation the position of points.
                        objs_2d[k]->sum_pos_3d += PointPosWorld;
                    }
                }
            }
        }
    }
} // AssociateObjAndPoints() END -----------------------------------


// BRIEF [EAO] associate objects with lines.
void Tracking::AssociateObjAndLines(vector<Object_2D *> objs_2d)
{
    // all lines in current frame.
    Eigen::MatrixXd AllLinesEigen = mCurrentFrame.all_lines_eigen;

    // step 1 make sure edges start from left to right.
    align_left_right_edges(AllLinesEigen);

    for(int i = 0; i < objs_2d.size(); i++)
    {
        Object_2D* obj = objs_2d[i];

        // step 2. expand the bounding box.
        double dLeftExpand = max(0.0, obj->mBox.x - 15.0);
        double dRightExpand = min(mCurrentFrame.mColorImage.cols, obj->mBox.x + obj->mBox.width + 15);
        double dTopExpand = max(0.0, obj->mBox.y - 15.0);
        double dBottomExpand = min(mCurrentFrame.mColorImage.rows, obj->mBox.y + obj->mBox.height + 15);
        Vector2d ExpanLeftTop = Vector2d(dLeftExpand, dTopExpand);			// lefttop.
		Vector2d ExpanRightBottom = Vector2d(dRightExpand, dBottomExpand);  // rightbottom.

        // step 3. associate object with lines.
        Eigen::MatrixXd ObjectLines(AllLinesEigen.rows(),AllLinesEigen.cols()); 
		int nInsideLinesNum = 0;
		for (int line_id = 0; line_id < AllLinesEigen.rows(); line_id++)
        {
            // check endpoints of the lines, whether inside the box.
            if (check_inside_box(   AllLinesEigen.row(line_id).head<2>(), 
                                    ExpanLeftTop, 
                                    ExpanRightBottom ))
            {
                if(check_inside_box(AllLinesEigen.row(line_id).tail<2>(),
                                    ExpanLeftTop, 
                                    ExpanRightBottom ))
                {
                    ObjectLines.row(nInsideLinesNum) = AllLinesEigen.row(line_id);
                    nInsideLinesNum++;
                }
            }
        }

        // step 4. merge lines.
        double pre_merge_dist_thre = 20; 
		double pre_merge_angle_thre = 5; 
		double edge_length_threshold = 30;
	    MatrixXd ObjectLinesAfterMerge;
		merge_break_lines(	ObjectLines.topRows(nInsideLinesNum), 
							ObjectLinesAfterMerge, 		// output lines after merge.
							pre_merge_dist_thre,		// the distance threshold between two line, 20 pixels.
							pre_merge_angle_thre, 		// angle threshold between two line, 5°.
							edge_length_threshold);		// length threshold, 30 pixels.

        // step 5. save object lines.
        obj->mObjLinesEigen = ObjectLinesAfterMerge;
        mCurrentFrame.vObjsLines.push_back(ObjectLinesAfterMerge);
    }
} // AssociateObjAndLines() END ----------------------------------.


// BRIEF [EAO] Initialize the object map.
void Tracking::InitObjMap(vector<Object_2D *> objs_2d)
{
    int nGoodObjId = -1;        // object id.
    for (auto &obj : objs_2d)
    {
        // Initialize the object map need enough points.
        if (obj->Obj_c_MapPonits.size() < 10)
        {
            obj->few_mappoint = true;
            obj->current = false;
            continue;
        }

        nGoodObjId++;

        mbObjectIni = true;
        mnObjectIniFrameID = mCurrentFrame.mnId;

        // Create an object in the map.
        Object_Map *ObjectMapSingle = new Object_Map;
        ObjectMapSingle->mObjectFrame.push_back(obj);   // 2D objects in each frame associated with this 3D map object.
        ObjectMapSingle->mnId = nGoodObjId;             // 3d objects in the map.
        ObjectMapSingle->mnClass = obj->_class_id;      // object class.
        ObjectMapSingle->mnConfidence = 1;              // object confidence = mObjectFrame.size().
        ObjectMapSingle->mbFirstObserve = true;                 // the object was observed for the first time.
        ObjectMapSingle->mnAddedID = mCurrentFrame.mnId;        // added id.
        ObjectMapSingle->mnLastAddID = mCurrentFrame.mnId;      // last added id.
        ObjectMapSingle->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
        ObjectMapSingle->mLastRect = obj->mBoxRect;             // last rect.
        // ObjectMapSingle->mPredictRect = obj->mBoxRect;       // for iou.
        ObjectMapSingle->msFrameId.insert(mCurrentFrame.mnId);  // no used in this version.
        ObjectMapSingle->mSumPointsPos = obj->sum_pos_3d;       // accumulated coordinates of object points.
        ObjectMapSingle->mCenter3D = obj->_Pos;                 // 3d centre.
        obj->mAssMapObjCenter = obj->_Pos;                      // for optimization, no used in this version.

        // add properties of the point and save it to the object.
        for (size_t i = 0; i < obj->Obj_c_MapPonits.size(); i++)
        {
            MapPoint *pMP = obj->Obj_c_MapPonits[i];

            pMP->object_id = ObjectMapSingle->mnId;                            
            pMP->object_class = ObjectMapSingle->mnClass;                      
            pMP->object_id_vector.insert(make_pair(ObjectMapSingle->mnId, 1)); // the point is first observed by the object.

            if (ObjectMapSingle->mbFirstObserve == true)
                pMP->First_obj_view = true; 

            // save to the object.
            ObjectMapSingle->mvpMapObjectMappoints.push_back(pMP);
        }

        // 2d object.
        obj->mnId = ObjectMapSingle->mnId;
        obj->mnWhichTime = ObjectMapSingle->mnConfidence;
        obj->current = true;

        // save this 2d object to current frame (associates with a 3d object in the map).
        mCurrentFrame.mvObjectFrame.push_back(obj);
        //mCurrentFrame.mvLastObjectFrame.push_back(obj);    
        //mCurrentFrame.mvLastLastObjectFrame.push_back(obj); 

        // todo: save to key frame.

        // updata map.
        ObjectMapSingle->ComputeMeanAndStandard();
        mpMap->mvObjectMap.push_back(ObjectMapSingle);
    }
} // initialize the object map. END -----------------------------------------------------


// BRIEF [EAO] project points to image.
cv::Point2f Tracking::WorldToImg(cv::Mat &PointPosWorld)
{
    // world.
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // camera.
    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

    const float xc = PointPosCamera.at<float>(0);
    const float yc = PointPosCamera.at<float>(1);
    const float invzc = 1.0 / PointPosCamera.at<float>(2);

    // image.
    float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
    float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

    return cv::Point2f(u, v);
} // WorldToImg(cv::Mat &PointPosWorld) END ------------------------------


// BRIEF [EAO] Estimate object orientation.
void Tracking::SampleObjYaw(Object_Map* objMap)
{
    // demo 1: compare the results without estimating the orientation.
    if((mflag == "None") || (mflag == "iForest"))
        return;

    int numMax = 0;
    float fError = 0.0;
    float fErrorYaw;
    float minErrorYaw = 360.0;
    float sampleYaw = 0.0;
    int nAllLineNum = objMap->mObjectFrame.back()->mObjLinesEigen.rows();

    for(int i = 0; i < 30; i++)
    {
        // initial angle.
        float roll, pitch, yaw;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        float error = 0.0;
        float errorYaw = 0.0;

        // 1 -> 15: -45° - 0°
        // 16 -> 30: 0° - 45°
        if(i < 15)
            yaw = (0.0 - i*3.0)/180.0 * M_PI;
        else
            yaw = (0.0 + (i-15)*3.0)/180.0 * M_PI;

        // object pose in object frame. (Ryaw)
        float cp = cos(pitch);
        float sp = sin(pitch);
        float sr = sin(roll);
        float cr = cos(roll);
        float sy = sin(yaw);
        float cy = cos(yaw);
        Eigen::Matrix<double,3,3> REigen;
        REigen<<   cp*cy, (sr*sp*cy)-(cr*sy), (cr*sp*cy)+(sr*sy),
                cp*sy, (sr*sp*sy)+(cr*cy), (cr*sp*sy)-(sr*cy),
                    -sp,    sr*cp,              cr * cp;
        cv::Mat Ryaw = Converter::toCvMat(REigen);

        // 8 vertices of the 3D box, world --> object frame.
        cv::Mat corner_1 = Converter::toCvMat(objMap->mCuboid3D.corner_1_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_2 = Converter::toCvMat(objMap->mCuboid3D.corner_2_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_3 = Converter::toCvMat(objMap->mCuboid3D.corner_3_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_4 = Converter::toCvMat(objMap->mCuboid3D.corner_4_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_5 = Converter::toCvMat(objMap->mCuboid3D.corner_5_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_6 = Converter::toCvMat(objMap->mCuboid3D.corner_6_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_7 = Converter::toCvMat(objMap->mCuboid3D.corner_7_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_8 = Converter::toCvMat(objMap->mCuboid3D.corner_8_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);

        // rotate in object frame  + object frame --> world frame.
        corner_1 = Ryaw * corner_1 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_2 = Ryaw * corner_2 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_3 = Ryaw * corner_3 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_4 = Ryaw * corner_4 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_5 = Ryaw * corner_5 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_6 = Ryaw * corner_6 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_7 = Ryaw * corner_7 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_8 = Ryaw * corner_8 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);

        // step 1. project 8 vertices to image.
        cv::Point2f point1, point2, point3, point4, point5, point6, point7, point8;
        point1 = WorldToImg(corner_1);
        point2 = WorldToImg(corner_2);
        point3 = WorldToImg(corner_3);
        point4 = WorldToImg(corner_4);
        point5 = WorldToImg(corner_5);
        point6 = WorldToImg(corner_6);
        point7 = WorldToImg(corner_7);
        point8 = WorldToImg(corner_8);

        // step 2. angle of 3 edges(lenth, width, height).
        float angle1;
        float angle2;
        float angle3;
        // left -> right.
        if(point6.x > point5.x)
            angle1 = atan2(point6.y - point5.y, point6.x - point5.x);
        else
            angle1 = atan2(point5.y - point6.y, point5.x - point6.x);
        float lenth1 = sqrt((point6.y - point5.y) * (point6.y - point5.y) + (point6.x - point5.x) * (point6.x - point5.x));

        if(point7.x > point6.x)
            angle2 = atan2(point7.y - point6.y, point7.x - point6.x);
        else
            angle2 = atan2(point6.y - point7.y, point6.x - point7.x);
        float lenth2 = sqrt((point7.y - point6.y) * (point7.y - point6.y) + (point7.x - point6.x) * (point7.x - point6.x));

        if(point6.x > point2.x)
            angle3 = atan2(point6.y - point2.y, point6.x - point2.x);
        else
            angle3 = atan2(point2.y - point6.y, point2.x - point6.x);
        float lenth3 = sqrt((point6.y - point2.y) * (point6.y - point2.y) + (point6.x - point2.x) * (point6.x - point2.x));

        // step 3. compute angle between detected lines and cube edges.
        int num = 0;
        for(int line_id = 0; line_id < objMap->mObjectFrame.back()->mObjLinesEigen.rows(); line_id++)
        {
            // angle of detected lines.
            double x1 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 0);
            double y1 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 1);
            double x2 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 2);
            double y2 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 3);
            float angle = atan2(y2 - y1, x2 - x1);

            // lenth.
            float lenth = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));

            // angle between line and 3 edges.
            float dis_angle1 = abs(angle * 180/M_PI - angle1 * 180/M_PI);
            float dis_angle2 = abs(angle * 180/M_PI - angle2 * 180/M_PI);
            float dis_angle3 = abs(angle * 180/M_PI - angle3 * 180/M_PI);

            float th = 5.0;             // threshold of the angle.
            if(objMap->mnClass == 56)   // chair.
            {
                if((dis_angle2 < th) || (dis_angle3 < th))    
                    num++;
                if(dis_angle1 < th)
                {
                    num+=3;
                }
            }
            else
            {
                // the shortest edge is lenth1.
                if( min(min(lenth1, lenth2), lenth3) == lenth1)
                {
                    // error with other two edges.
                    if((dis_angle2 < th) || (dis_angle3 < th))
                    {
                        num++;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }

                    // angle error.
                    errorYaw+=min(dis_angle2, dis_angle3);
                }
                // the shortest edge is lenth2.
                if( min(min(lenth1, lenth2), lenth3) == lenth2)
                {
                    if((dis_angle1 < th) || (dis_angle3 < th))    
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }
                    errorYaw+=min(dis_angle3, dis_angle1);
                }
                // the shortest edge is lenth3.
                if( min(min(lenth1, lenth2), lenth3) == lenth3)
                {
                    if((dis_angle1 < th) || (dis_angle2 < th))  
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                    }
                    errorYaw+=min(dis_angle2, dis_angle1);
                }
            }
        }
        if(num == 0)
        {
            num = 1;
            errorYaw = 10.0;
        }

        // record the angle with max number parallel lines.
        if(num > numMax)
        {
            numMax = num;
            sampleYaw = yaw;

            fError = error; // no used in this version.
            // average angle error.
            fErrorYaw = (errorYaw/(float)num)/10.0;
        }
    }

    // step 4. scoring.
    float fScore;
    fScore = ((float)numMax / (float)nAllLineNum) * (1.0 - 0.1 * fErrorYaw);
    if(isinf(fScore))
        fScore = 0.0;

    // measurement： yaw, times, score, angle, angle error.
    Vector5f AngleTimesAndScore;
    AngleTimesAndScore[0] = sampleYaw;
    AngleTimesAndScore[1] = 1.0;
    AngleTimesAndScore[2] = fScore;
    AngleTimesAndScore[3] = fError;     // no used in this version.
    AngleTimesAndScore[4] = fErrorYaw;

    // update multi-frame measurement.
    bool bNewMeasure = true;
    for (auto &row : objMap->mvAngleTimesAndScore)
    {
        if(row[0] == AngleTimesAndScore[0])
        {   
            row[1] += 1.0;
            row[2] = AngleTimesAndScore[2] * (1/row[1]) + row[2] * (1 - 1/row[1]);
            row[3] = AngleTimesAndScore[3] * (1/row[1]) + row[3] * (1 - 1/row[1]);
            row[4] = AngleTimesAndScore[4] * (1/row[1]) + row[4] * (1 - 1/row[1]);

            bNewMeasure = false;
        }
    }
    if(bNewMeasure == true)
    {
        objMap->mvAngleTimesAndScore.push_back(AngleTimesAndScore);
    }

    // step 5. rank.
    index = 1;
    std::sort(objMap->mvAngleTimesAndScore.begin(),objMap->mvAngleTimesAndScore.end(),VIC);
    // for (auto &row : objMap->mvAngleTimesAndScore)
    // {
    //     std::cout << row[0] * 180.0 / M_PI  << "\t" <<  row[1] << "\t" <<  row[2] << std::endl;
    // }
    // the best yaw.
    int best_num = 0;
    float best_score = 0;
    for(int i = 0; i < std::min(3, (int)objMap->mvAngleTimesAndScore.size()); i++)
    {
        float fScore = objMap->mvAngleTimesAndScore[i][2];
        if(fScore >= best_score)
        {
            best_score = fScore;
            best_num = i;
        }
    }

    // step 6. update object yaw.
    objMap->mCuboid3D.rotY = objMap->mvAngleTimesAndScore[best_num][0];
    objMap->mCuboid3D.mfErrorParallel = objMap->mvAngleTimesAndScore[best_num][3];
    objMap->mCuboid3D.mfErroeYaw = objMap->mvAngleTimesAndScore[best_num][4];
} // SampleObjYaw() END -------------------------------------------------------------------------


// BRIEF [EAO] project quadrics from world to image.
cv::Mat Tracking::DrawQuadricProject(cv::Mat &im,
                                     const cv::Mat &P,   // projection matrix.
                                     const cv::Mat &axe, // axis length.
                                     const cv::Mat &Twq, // object pose.
                                     int nClassid,
                                     bool isGT,
                                     int nLatitudeNum,
                                     int nLongitudeNum)
{
    // color.
    std::vector<cv::Scalar> colors = {  cv::Scalar(135,0,248),
                                        cv::Scalar(255,0,253),
                                        cv::Scalar(4,254,119),
                                        cv::Scalar(255,126,1),
                                        cv::Scalar(0,112,255),
                                        cv::Scalar(0,250,250),
                                        };

    // draw params
    cv::Scalar sc = colors[nClassid % 6];

    int nLineWidth = 2;

    // generate angluar grid -> xyz grid (vertical half sphere)
    vector<float> vfAngularLatitude;  // (-90, 90)
    vector<float> vfAngularLongitude; // [0, 180]
    cv::Mat pointGrid(nLatitudeNum + 2, nLongitudeNum + 1, CV_32FC4);

    for (int i = 0; i < nLatitudeNum + 2; i++)
    {
        float fThetaLatitude = -M_PI_2 + i * M_PI / (nLatitudeNum + 1);
        cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
        for (int j = 0; j < nLongitudeNum + 1; j++)
        {
            float fThetaLongitude = j * M_PI / nLongitudeNum;
            p[j][0] = axe.at<float>(0, 0) * cos(fThetaLatitude) * cos(fThetaLongitude);
            p[j][1] = axe.at<float>(1, 0) * cos(fThetaLatitude) * sin(fThetaLongitude);
            p[j][2] = axe.at<float>(2, 0) * sin(fThetaLatitude);
            p[j][3] = 1.;
        }
    }

    // draw latitude
    for (int i = 0; i < pointGrid.rows; i++)
    {
        cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
        // [0, 180]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    // draw longitude
    cv::Mat pointGrid_t = pointGrid.t();
    for (int i = 0; i < pointGrid_t.rows; i++)
    {
        cv::Vec4f *p = pointGrid_t.ptr<cv::Vec4f>(i);
        // [0, 180]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    return im;
} // DrawQuadricProject() END  -----------------------------------------------------------------------------------------------------

} // namespace ORB_SLAM2
