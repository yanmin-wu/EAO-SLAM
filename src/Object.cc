/*
 * =============================================
 *      Filename:  Object.cc
 *
 *      Description:
 *      Version: 1.0
 *      Created: 09/19/2019
 *      Author: Yanmin Wu
 *      E-mail: wuyanminmax@gmail.com
 * ==============================================
 */

#include "Object.h"

// cube slam
#include "detect_3d_cuboid/object_3d_util.h"
#include "detect_3d_cuboid/matrix_utils.h"

#include <iostream>
#include <stdint.h>
#include <random>
#include <vector>
#include <string>
#include "isolation_forest.h"
#include <math.h>
#include "Converter.h"

namespace ORB_SLAM2
{

bool biForest = true;


// BRIEF copy box to object_2d.
void Object_2D::CopyBoxes(const BoxSE &box)
{
    // class, probability.
    _class_id = box.m_class;
    mScore = box.m_score;

    // box.
    left = box.x;
    right = box.x + box.width;
    top = box.y;
    bottom = box.y + box.height;

    // widht, height.
    mWidth = box.width;
    mHeight = box.height;

    // 2d center.
    box_center_2d = cv::Point2f(box.x + box.width / 2, box.y + box.height / 2);

    // opencv Rect format.
    mBoxRect = cv::Rect(box.x, box.y, box.width, box.height);

    // original box format.
    mBox = box;
} // CopyBoxes().


// BRIEF compute the mean and standard deviation of object points in current frame.
void Object_2D::ComputeMeanAndStandardFrame()
{
    // remove bad points.
    vector<MapPoint *>::iterator pMP;
    for (pMP = Obj_c_MapPonits.begin();
         pMP != Obj_c_MapPonits.end();)
    {
        cv::Mat pos = (*pMP)->GetWorldPos();

        if ((*pMP)->isBad())
        {
            pMP = Obj_c_MapPonits.erase(pMP);
            sum_pos_3d -= pos;
        }
        else
            ++pMP;
    }

    // mean(3d center)
    _Pos = sum_pos_3d / (Obj_c_MapPonits.size());

    // standard deviation in 3 directions.
    float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    for (size_t i = 0; i < Obj_c_MapPonits.size(); i++)
    {
        MapPoint *pMP = Obj_c_MapPonits[i];
        if (pMP->isBad())
            continue;

        cv::Mat pos = pMP->GetWorldPos();
        cv::Mat pos_ave = _Pos;

        sum_x2 += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2 += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2 += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));
    }
    mStandar_x = sqrt(sum_x2 / (Obj_c_MapPonits.size()));
    mStandar_y = sqrt(sum_y2 / (Obj_c_MapPonits.size()));
    mStandar_z = sqrt(sum_z2 / (Obj_c_MapPonits.size()));
} // ComputeMeanAndStandardFrame().


// BRIEF Remove outliers (camera frame) by boxplot.
void Object_2D::RemoveOutliersByBoxPlot(Frame &mCurrentFrame)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // world -> camera.
    vector<float> x_c;
    vector<float> y_c;
    vector<float> z_c;
    for (size_t i = 0; i < Obj_c_MapPonits.size(); i++)
    {
        MapPoint *pMP = Obj_c_MapPonits[i];

        cv::Mat PointPosWorld = pMP->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        x_c.push_back(PointPosCamera.at<float>(0));
        y_c.push_back(PointPosCamera.at<float>(1));
        z_c.push_back(PointPosCamera.at<float>(2));
    }

    // sort.
    sort(x_c.begin(), x_c.end());
    sort(y_c.begin(), y_c.end());
    sort(z_c.begin(), z_c.end());

    if ((z_c.size() / 4 <= 0) || (z_c.size() * 3 / 4 >= z_c.size() - 1))
        return;

    float Q1 = z_c[(int)(z_c.size() / 4)];
    float Q3 = z_c[(int)(z_c.size() * 3 / 4)];
    float IQR = Q3 - Q1;

    float min_th = Q1 - 1.5 * IQR;
    float max_th = Q3 + 1.5 * IQR;

    vector<MapPoint *>::iterator pMP;
    for (pMP = Obj_c_MapPonits.begin();
         pMP != Obj_c_MapPonits.end();)
    {
        cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        float z = PointPosCamera.at<float>(2);

        if (z > max_th)
            pMP = Obj_c_MapPonits.erase(pMP);   // remove.
        else
            ++pMP;
    }

    this->ComputeMeanAndStandardFrame();
} // RemoveDeepErrorPoints().


// BRIEF 2d objects (in the frame) associate with 3d objects (in the map).
void Object_2D::ObjectDataAssociation(Map *mpMap, Frame &mCurrentFrame, cv::Mat &image, string &flag)
{
    if(flag == "None")
        biForest = false;

    bool old = false;   // no used.

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    cv::Rect RectCurrent = mBoxRect;    // object bounding box in current frame.
    cv::Rect RectPredict;               // predicted bounding box according to last frame and next to last frame.
    cv::Rect RectProject;               // bounding box constructed by projecting points.
    float IouMax = 0;
    bool bAssoByIou = false;            // whether associated by IoU.
    int nAssoByIouId = -1;              // the associated map object ID.
    int IouMaxObjID = -1;               // temporary variable.
    float IouThreshold = 0.5;           // IoU threshold.

    // ****************************************************
    //         STEP 1. IoU data association.              *
    // ****************************************************
    if((flag != "NA") && (flag != "NP"))
    {
        for (int i = 0; i < (int)mpMap->mvObjectMap.size(); i++)
        {
            if (_class_id != mpMap->mvObjectMap[i]->mnClass)
                continue;

            if (mpMap->mvObjectMap[i]->bBadErase)
                continue;

            if (mpMap->mvObjectMap[i]->mnLastAddID == mCurrentFrame.mnId - 1)
            {
                // step 1.1 predict object bounding box according to last frame and next to last frame.
                if (mpMap->mvObjectMap[i]->mnLastLastAddID == mCurrentFrame.mnId - 2)
                {
                    // left-top.
                    float left_top_x = mpMap->mvObjectMap[i]->mLastRect.x * 2 - mpMap->mvObjectMap[i]->mLastLastRect.x;
                    if (left_top_x < 0)
                        left_top_x = 0;
                    float left_top_y = mpMap->mvObjectMap[i]->mLastRect.y * 2 - mpMap->mvObjectMap[i]->mLastLastRect.y;
                    if (left_top_y < 0)
                        left_top_y = 0;

                    // right-bottom.
                    float right_down_x = (mpMap->mvObjectMap[i]->mLastRect.x + mpMap->mvObjectMap[i]->mLastRect.width) * 2 - (mpMap->mvObjectMap[i]->mLastLastRect.x + mpMap->mvObjectMap[i]->mLastLastRect.width);
                    if (left_top_x > image.cols)
                        right_down_x = image.cols;
                    float right_down_y = (mpMap->mvObjectMap[i]->mLastRect.y + mpMap->mvObjectMap[i]->mLastRect.height) * 2 - (mpMap->mvObjectMap[i]->mLastLastRect.y + mpMap->mvObjectMap[i]->mLastLastRect.height);
                    if (left_top_y > image.rows)
                        right_down_y = image.rows;

                    float width = right_down_x - left_top_x;
                    float height = right_down_y - left_top_y;

                    // predicted bounding box.
                    RectPredict = cv::Rect(left_top_x, left_top_y, width, height);

                    // If two consecutive frames are observed, increase the threshold.
                    IouThreshold = 0.6;
                }
                else
                    RectPredict = mpMap->mvObjectMap[i]->mLastRect;

                // step 1.2 compute IoU, record the max IoU and the map object ID.
                float Iou = Converter::bboxOverlapratio(RectCurrent, RectPredict);
                if ((Iou > IouThreshold) && Iou > IouMax)
                {
                    IouMax = Iou;
                    IouMaxObjID = i;
                }
            }
        }
        // step 1.3 if the association is successful, update the map object.
        if ((IouMax > 0) && (IouMaxObjID >= 0))
        {
            // update.
            bool bFlag = mpMap->mvObjectMap[IouMaxObjID]->DataAssociateUpdate(this, mCurrentFrame, image, 1);

            if (bFlag)
            {
                bAssoByIou = true;              // associated by IoU.
                nAssoByIouId = IouMaxObjID;     // associated map object id.
            }
        }
    }
    // Iou data association END ----------------------------------------------------------------------------


    // *************************************************
    //      STEP 2. Nonparametric data association     *
    // *************************************************
    bool bAssoByNp = false;
    int nAssoByNPId = -1;
    vector<int> vObjByNPId;     // potential associated objects.
    if((flag != "NA") && (flag != "IoU"))
    {
        for (int i = (int)mpMap->mvObjectMap.size() - 1; (i >= 0) && (old == false); i--)
        {
            if (_class_id != mpMap->mvObjectMap[i]->mnClass)
                continue;

            if (mpMap->mvObjectMap[i]->bBadErase)
                continue;

            // step 2.1 nonparametric test.
            int nFlag = this->NoParaDataAssociation(mpMap->mvObjectMap[i], mCurrentFrame, image);

            if (nFlag == 0) // 0: skip the nonparametric test and continue with the subsequent t-test.
                break;
            if (nFlag == 2) // 2: association failed, compare next object.
                continue;
            else if (nFlag == 1) // 1: association succeeded, but there may be more than one.
                vObjByNPId.push_back(i);
        }

        // step 2.2 update data association and record potential associated objects.
        if (vObjByNPId.size() >= 1)
        {
            // case 1: if associated by IoU, the objects judged by nonparametric-test are marked as potential association objects.
            if (bAssoByIou)
            {
                for (int i = 0; i < vObjByNPId.size(); i++)
                {
                    if (vObjByNPId[i] == nAssoByIouId)
                        continue;

                    // Record potential association objects and potential association times.
                    map<int, int>::iterator sit;
                    sit = mpMap->mvObjectMap[nAssoByIouId]->mReObj.find(mpMap->mvObjectMap[vObjByNPId[i]]->mnId);
                    if (sit != mpMap->mvObjectMap[nAssoByIouId]->mReObj.end())
                    {
                        int sit_sec = sit->second;
                        mpMap->mvObjectMap[nAssoByIouId]->mReObj.erase(mpMap->mvObjectMap[vObjByNPId[i]]->mnId);
                        mpMap->mvObjectMap[nAssoByIouId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByNPId[i]]->mnId, sit_sec + 1));
                    }
                    else
                        mpMap->mvObjectMap[nAssoByIouId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByNPId[i]]->mnId, 1));
                }
            }
            // case 2: if association failed by IoU, 
            else
            {
                for (int i = 0; i < vObjByNPId.size(); i++)
                {
                    // update.
                    bool bFlag = mpMap->mvObjectMap[vObjByNPId[i]]->DataAssociateUpdate(this, mCurrentFrame, image, 2); // 2: NP.

                    // if association successful, other objects are marked as potential association objects.
                    if (bFlag)
                    {
                        bAssoByNp = true;               // associated by NP.
                        nAssoByNPId = vObjByNPId[i];    // associated map object id.

                        if (vObjByNPId.size() > i + 1)
                        {
                            for (int j = i + 1; j < vObjByNPId.size(); j++)
                            {
                                // Record potential association objects and potential association times.
                                map<int, int>::iterator sit;
                                sit = mpMap->mvObjectMap[vObjByNPId[i]]->mReObj.find(mpMap->mvObjectMap[vObjByNPId[j]]->mnId);
                                if (sit != mpMap->mvObjectMap[vObjByNPId[i]]->mReObj.end())
                                {
                                    int sit_sec = sit->second;
                                    mpMap->mvObjectMap[vObjByNPId[i]]->mReObj.erase(mpMap->mvObjectMap[vObjByNPId[j]]->mnId);
                                    mpMap->mvObjectMap[vObjByNPId[i]]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByNPId[j]]->mnId, sit_sec + 1));
                                }
                                else
                                    mpMap->mvObjectMap[vObjByNPId[i]]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByNPId[j]]->mnId, 1));
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    // Nonparametric data association END --------------------------------------------------------------------------------------------------------

    if (old == false)
    {
        // ****************************************************
        //         STEP 3. Projected box data association     *
        // ****************************************************
        bool bAssoByProject = false;
        int nAssoByProId = -1;
        vector<int> vObjByProIouId;
        
        if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
        {
            float fIouMax = 0.0;
            int ProIouMaxObjId = -1;
            for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
            {
                if (_class_id != mpMap->mvObjectMap[i]->mnClass)
                    continue;

                if (mpMap->mvObjectMap[i]->bBadErase)
                    continue;

                int df = (int)mpMap->mvObjectMap[i]->mObjectFrame.size();

                if ((Obj_c_MapPonits.size() >= 10) && (df > 8))
                    continue;

                // step 3.1 compute IoU with bounding box constructed by projecting points.
                float fIou = Converter::bboxOverlapratio(RectCurrent, mpMap->mvObjectMap[i]->mRectProject);
                float fIou2 = Converter::bboxOverlapratio(mRectFeaturePoints, mpMap->mvObjectMap[i]->mRectProject);
                fIou = max(fIou, fIou2);

                // record the max IoU and map object id.
                if ((fIou >= 0.25) && (fIou > fIouMax))
                {
                    fIouMax = fIou;
                    ProIouMaxObjId = i;
                    vObjByProIouId.push_back(i);
                }
            }
            // step 3.2 update data association and record potential associated objects.
            if (fIouMax >= 0.25)
            {
                sort(vObjByProIouId.begin(), vObjByProIouId.end());

                if (bAssoByIou || bAssoByNp)
                {
                    for (int j = vObjByProIouId.size() - 1; j >= 0; j--)
                    {
                        int ReId;
                        if (bAssoByIou)
                            ReId = nAssoByIouId;
                        if (bAssoByNp)
                            ReId = nAssoByNPId;

                        if (vObjByProIouId[j] == ReId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = mpMap->mvObjectMap[ReId]->mReObj.find(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
                        if (sit != mpMap->mvObjectMap[ReId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            mpMap->mvObjectMap[ReId]->mReObj.erase(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
                            mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, sit_sec + 1));
                        }
                        else
                            mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, 1));
                    }
                }
                else
                {
                    // update.
                    bool bFlag = mpMap->mvObjectMap[ProIouMaxObjId]->DataAssociateUpdate(this, mCurrentFrame, image, 4); // 4: project iou.

                    // association succeeded.
                    if (bFlag)
                    {
                        bAssoByProject = true;          // associated by projecting box.
                        nAssoByProId = ProIouMaxObjId;  // associated map object id.
                    }

                    for (int j = vObjByProIouId.size() - 1; j >= 0; j--)
                    {
                        if (vObjByProIouId[j] == ProIouMaxObjId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.find(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
                        if (sit != mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.erase(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
                            mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, sit_sec + 1));
                        }
                        else
                            mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, 1));
                    }
                }
            }
        }
        // Projected box data association END ---------------------------------------------------------------------------------------

        // ************************************************
        //          STEP 4. t-test data association       *
        // ************************************************
        // step 4.1 Read t-distribution boundary value.
        float tTestData[122][9] = {0};
        ifstream infile;
        infile.open("./data/t_test.txt");
        for (int i = 0; i < 122; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                infile >> tTestData[i][j];
            }
        }
        infile.close();
        
        // step 4.2 t-test.
        bool bAssoByT = false;
        int nAssoByTId = -1;
        vector<int> vObjByTId;
        vector<int> vObjByTIdLower; // potential association.
        if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
        {
            for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
            {
                if (_class_id != mpMap->mvObjectMap[i]->mnClass)
                    continue;

                if (mpMap->mvObjectMap[i]->bBadErase)
                    continue;

                // t-test results in 3 directions.
                float t_test;
                float t_test_x, t_test_y, t_test_z;

                // Degrees of freedom.
                int df = (int)mpMap->mvObjectMap[i]->mObjectFrame.size();

                if (df <= 8)
                    continue;

                // Iou.
                float fIou = Converter::bboxOverlapratio(RectCurrent, mpMap->mvObjectMap[i]->mRectProject);
                float fIou2 = Converter::bboxOverlapratio(mRectFeaturePoints, mpMap->mvObjectMap[i]->mRectProject);
                fIou = max(fIou, fIou2);

                // The distance from points to the object center.
                float dis_x, dis_y, dis_z;
                dis_x = abs(mpMap->mvObjectMap[i]->mCenter3D.at<float>(0, 0) - _Pos.at<float>(0, 0));
                dis_y = abs(mpMap->mvObjectMap[i]->mCenter3D.at<float>(1, 0) - _Pos.at<float>(1, 0));
                dis_z = abs(mpMap->mvObjectMap[i]->mCenter3D.at<float>(2, 0) - _Pos.at<float>(2, 0));

                // t-test.
                t_test_x = dis_x / (mpMap->mvObjectMap[i]->mCenterStandar_x / sqrt(df));
                t_test_y = dis_y / (mpMap->mvObjectMap[i]->mCenterStandar_y / sqrt(df));
                t_test_z = dis_z / (mpMap->mvObjectMap[i]->mCenterStandar_z / sqrt(df));

                // Satisfy t test.  // 5->0.05.
                if ((t_test_x < tTestData[min((df - 1), 121)][5]) &&
                    (t_test_y < tTestData[min((df - 1), 121)][5]) &&
                    (t_test_z < tTestData[min((df - 1), 121)][5]))
                {
                    vObjByTId.push_back(i);
                }
                // If the T-test is not satisfied, but the IOU is large, reducing the significance.
                else if (fIou > 0.25)
                {
                    if ((t_test_x < tTestData[min((df - 1), 121)][8]) &&
                        (t_test_y < tTestData[min((df - 1), 121)][8]) &&
                        (t_test_z < tTestData[min((df - 1), 121)][8]))
                    {
                        vObjByTId.push_back(i);
                    }

                    else if ((fIou > 0.25) && ((t_test_x + t_test_y + t_test_z) / 3 < 10))
                    {
                        vObjByTId.push_back(i);
                    }
                    else
                    {
                        vObjByTIdLower.push_back(i);
                    }
                }
                else if ((t_test_x + t_test_y + t_test_z) / 3 < 4)
                {
                    mpMap->mvObjectMap[i]->ComputeProjectRectFrame(image, mCurrentFrame);

                    float fIou_force = Converter::bboxOverlapratio(RectCurrent, mpMap->mvObjectMap[i]->mRectProject);
                    float fIou2_force = Converter::bboxOverlapratio(mRectFeaturePoints, mpMap->mvObjectMap[i]->mRectProject);
                    fIou_force = max(fIou_force, fIou2_force);

                    if (fIou_force > 0.25)
                        vObjByTIdLower.push_back(i);
                }
            }

            // step 4.2 update data association and record potential associated objects.
            if (bAssoByIou || bAssoByNp || bAssoByProject)
            {
                int ReId;
                if (bAssoByIou)
                    ReId = nAssoByIouId;
                if (bAssoByNp)
                    ReId = nAssoByNPId;
                if (bAssoByProject)
                    ReId = nAssoByProId;

                if (vObjByTId.size() >= 1)
                {
                    for (int j = 0; j < vObjByTId.size(); j++)
                    {
                        if (vObjByTId[j] == ReId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = mpMap->mvObjectMap[ReId]->mReObj.find(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
                        if (sit != mpMap->mvObjectMap[ReId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            mpMap->mvObjectMap[ReId]->mReObj.erase(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
                            mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, sit_sec + 1));
                        }
                        else
                            mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, 1));
                    }
                }

                if (vObjByTIdLower.size() >= 0)
                {
                    for (int j = 0; j < vObjByTIdLower.size(); j++)
                    {
                        if (vObjByTIdLower[j] == ReId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = mpMap->mvObjectMap[ReId]->mReObj.find(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
                        if (sit != mpMap->mvObjectMap[ReId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            mpMap->mvObjectMap[ReId]->mReObj.erase(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
                            mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, sit_sec + 1));
                        }
                        else
                            mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, 1));
                    }
                }
            }
            else
            {
                if (vObjByTId.size() >= 1)
                {
                    for (int i = 0; i < vObjByTId.size(); i++)
                    {
                        bool bFlag = mpMap->mvObjectMap[vObjByTId[i]]->DataAssociateUpdate(this, mCurrentFrame, image, 3); // 3 是指 T 方法.

                        if (bFlag)
                        {
                            bAssoByT = true;
                            nAssoByTId = vObjByTId[i];

                            if (vObjByTId.size() > i)
                            {
                                for (int j = i + 1; j < vObjByTId.size(); j++)
                                {
                                    map<int, int>::iterator sit;
                                    sit = mpMap->mvObjectMap[nAssoByTId]->mReObj.find(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
                                    if (sit != mpMap->mvObjectMap[nAssoByTId]->mReObj.end())
                                    {
                                        int sit_sec = sit->second;
                                        mpMap->mvObjectMap[nAssoByTId]->mReObj.erase(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
                                        mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, sit_sec + 1));
                                    }
                                    else
                                        mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, 1));
                                }
                            }

                            if (vObjByTIdLower.size() >= 0)
                            {
                                for (int j = 0; j < vObjByTIdLower.size(); j++)
                                {
                                    if (vObjByTIdLower[j] == nAssoByTId)
                                        continue;

                                    map<int, int>::iterator sit;
                                    sit = mpMap->mvObjectMap[nAssoByTId]->mReObj.find(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
                                    if (sit != mpMap->mvObjectMap[nAssoByTId]->mReObj.end())
                                    {
                                        int sit_sec = sit->second;
                                        mpMap->mvObjectMap[nAssoByTId]->mReObj.erase(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
                                        mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, sit_sec + 1));
                                    }
                                    else
                                        mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, 1));
                                }
                            }

                            break; 
                        }
                    }
                }
            }
        }
        // t-test data association END ---------------------------------------------------------------------------------------

        // *************************************************
        //             STEP 4. create a new object         *
        // *************************************************
        if (bAssoByIou || bAssoByNp || bAssoByProject || bAssoByT)
            return;

        // If the object appears at the edge of the image, ignore.
        if ((this->mBox.x < 10) || (this->mBox.y < 10) ||
            (this->mBox.x + this->mBox.width > image.cols - 10) ||
            (this->mBox.y + this->mBox.height > image.rows - 10))
        {
            this->bad = true;
            return;
        }

        // create a 3d object in the map.
        Object_Map *ObjectMapSingle = new Object_Map;
        ObjectMapSingle->mObjectFrame.push_back(this);     
        ObjectMapSingle->mnId = mpMap->mvObjectMap.size(); 
        ObjectMapSingle->mnClass = _class_id;             
        ObjectMapSingle->mnConfidence = 1;              
        ObjectMapSingle->mbFirstObserve = true;            
        ObjectMapSingle->mnAddedID = mCurrentFrame.mnId;
        ObjectMapSingle->mnLastAddID = mCurrentFrame.mnId;
        ObjectMapSingle->mnLastLastAddID = mCurrentFrame.mnId;
        ObjectMapSingle->mLastRect = mBoxRect;                 
        ObjectMapSingle->msFrameId.insert(mCurrentFrame.mnId); 
        ObjectMapSingle->mSumPointsPos = sum_pos_3d;           
        ObjectMapSingle->mCenter3D = _Pos;
        this->mAssMapObjCenter = this->_Pos;

        // add properties of the point and save it to the object.
        for (size_t i = 0; i < Obj_c_MapPonits.size(); i++)
        {
            MapPoint *pMP = Obj_c_MapPonits[i];

            pMP->object_id = ObjectMapSingle->mnId;                           
            pMP->object_class = ObjectMapSingle->mnClass;                    
            pMP->object_id_vector.insert(make_pair(ObjectMapSingle->mnId, 1));

            if (ObjectMapSingle->mbFirstObserve == true) 
                pMP->First_obj_view = true;

            // save to the object.
            ObjectMapSingle->mvpMapObjectMappoints.push_back(pMP);
        }

        mnId = ObjectMapSingle->mnId;
        mnWhichTime = ObjectMapSingle->mnConfidence;
        current = true;

        // save this 2d object to current frame (associates with a 3d object in the map).
        mCurrentFrame.mvObjectFrame.push_back(this);
        mCurrentFrame.AppearNewObject = true;

        // update object map.
        ObjectMapSingle->IsolationForestDeleteOutliers();
        ObjectMapSingle->ComputeMeanAndStandard();
        mpMap->mvObjectMap.push_back(ObjectMapSingle);
        // create a new object END ------------------------------------------------------
    }
} // ObjectDataAssociation() END --------------------------------------------------------


// BRIEF nonparametric test.
int Object_2D::NoParaDataAssociation(Object_Map *ObjectMapSingle, Frame &mCurrentFrame, cv::Mat &image)
{
    // step 1. sample size.
    // 2d object in the frame -- m.
    int m = (int)Obj_c_MapPonits.size();
    int OutPointNum1 = 0;
    for (int ii = 0; ii < (int)Obj_c_MapPonits.size(); ii++)
    {
        MapPoint *p1 = Obj_c_MapPonits[ii];
        if (p1->isBad() || p1->out_point)
        {
            OutPointNum1++;
            continue;
        }
    }
    m = m - OutPointNum1;

    // 3d object int the object map -- n.
    int n = (int)ObjectMapSingle->mvpMapObjectMappoints.size();
    int OutPointNum2 = 0;
    for (int ii = 0; ii < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); ii++) // 帧中物体.
    {
        MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[ii];
        if (p2->isBad() || p2->out_point)
        {
            OutPointNum2++;
            continue;
        }
    }
    n = n - OutPointNum2;

    if (m < 20)
        return 0;

    if (n < 20)
        return 2;

    // Homogenization to avoid too many points of map object; n = 2 * m.
    bool bSampleMapPoints = true;
    vector<float> x_pt_map_sample;
    vector<float> y_pt_map_sample;
    vector<float> z_pt_map_sample;
    if (bSampleMapPoints)
    {
        int step = 1;
        if (n > 3 * m)
        {
            n = 3 * m;
            step = (int)ObjectMapSingle->mvpMapObjectMappoints.size() / n;

            vector<float> x_pt;
            vector<float> y_pt;
            vector<float> z_pt;
            for (int jj = 0; jj < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); jj++)
            {
                MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[jj];
                if (p2->isBad() || p2->out_point)
                {
                    continue;
                }

                cv::Mat x3D2 = p2->GetWorldPos();
                x_pt.push_back(x3D2.at<float>(0, 0));
                y_pt.push_back(x3D2.at<float>(1, 0));
                z_pt.push_back(x3D2.at<float>(2, 0));
            }
            sort(x_pt.begin(), x_pt.end());
            sort(y_pt.begin(), y_pt.end());
            sort(z_pt.begin(), z_pt.end());

            for (int i = 0; i < x_pt.size(); i += step)
            {
                x_pt_map_sample.push_back(x_pt[i]);
                y_pt_map_sample.push_back(y_pt[i]);
                z_pt_map_sample.push_back(z_pt[i]);
            }
            n = x_pt_map_sample.size();
        }
        else
        {
            for (int jj = 0; jj < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); jj++) // 地图中物体.
            {
                MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[jj];
                if (p2->isBad() || p2->out_point)
                {
                    continue;
                }

                cv::Mat x3D2 = p2->GetWorldPos();
                x_pt_map_sample.push_back(x3D2.at<float>(0, 0));
                y_pt_map_sample.push_back(x3D2.at<float>(1, 0));
                z_pt_map_sample.push_back(x3D2.at<float>(2, 0));
            }

            n = x_pt_map_sample.size();
        }
    }

    float w_x_12 = 0.0;
    float w_y_12 = 0.0;
    float w_z_12 = 0.0;
    float w_x_21 = 0.0;
    float w_y_21 = 0.0;
    float w_z_21 = 0.0;
    float w_x_00 = 0.0;
    float w_y_00 = 0.0;
    float w_z_00 = 0.0;
    float w_x = 0.0;
    float w_y = 0.0;
    float w_z = 0.0;

    for (int ii = 0; ii < (int)Obj_c_MapPonits.size(); ii++)
    {
        MapPoint *p1 = Obj_c_MapPonits[ii];
        if (p1->isBad() || p1->out_point)
            continue;

        cv::Mat x3D1 = p1->GetWorldPos();
        double x1 = x3D1.at<float>(0, 0);
        double y1 = x3D1.at<float>(1, 0);
        double z1 = x3D1.at<float>(2, 0);

        if (!bSampleMapPoints)
        {
            for (int jj = 0; jj < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); jj++)
            {
                MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[jj];
                if (p2->isBad() || p2->out_point)
                    continue;

                cv::Mat x3D2 = p2->GetWorldPos();
                double x2 = x3D2.at<float>(0, 0);
                double y2 = x3D2.at<float>(1, 0);
                double z2 = x3D2.at<float>(2, 0);

                if (x1 > x2)
                    w_x_12++;
                else if (x1 < x2)
                    w_x_21++;
                else if (x1 == x2)
                    w_x_00++;

                if (y1 > y2)
                    w_y_12++;
                else if (y1 < y2)
                    w_y_21++;
                else if (y1 == y2)
                    w_y_00++;

                if (z1 > z2)
                    w_z_12++;
                else if (z1 < z2)
                    w_z_21++;
                else if (z1 == z2)
                    w_z_00++;
            }
        }

        if (bSampleMapPoints)
        {
            for (int jj = 0; jj < (int)x_pt_map_sample.size(); jj++)
            {
                double x2 = x_pt_map_sample[jj];
                double y2 = y_pt_map_sample[jj];
                double z2 = z_pt_map_sample[jj];

                if (x1 > x2)
                    w_x_12++;
                else if (x1 < x2)
                    w_x_21++;
                else if (x1 == x2)
                    w_x_00++;

                if (y1 > y2)
                    w_y_12++;
                else if (y1 < y2)
                    w_y_21++;
                else if (y1 == y2)
                    w_y_00++;

                if (z1 > z2)
                    w_z_12++;
                else if (z1 < z2)
                    w_z_21++;
                else if (z1 == z2)
                    w_z_00++;
            }
        }
    }

    // step 2. compute the rank sum.
    w_x = min(w_x_12 + m * (m + 1) / 2, w_x_21 + n * (n + 1) / 2) + w_x_00 / 2;
    w_y = min(w_y_12 + m * (m + 1) / 2, w_y_21 + n * (n + 1) / 2) + w_y_00 / 2;
    w_z = min(w_z_12 + m * (m + 1) / 2, w_z_21 + n * (n + 1) / 2) + w_z_00 / 2;

    // step 3. compute the critical value.
    float r1 = 0.5 * m * (m + n + 1) - 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96
    float r2 = 0.5 * m * (m + n + 1) + 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96

    // step 4. whether the 3 directions meet the nonparametric test.
    bool old_np = false;
    int add = 0;
    if (w_x > r1 && w_x < r2)
        add++;
    if (w_y > r1 && w_y < r2)
        add++;
    if (w_z > r1 && w_z < r2)
        add++;

    if (add == 3)
        old_np = true;  // Nonparametric Association succeeded.

    if (old_np == 1)
        return 1;       // success.
    else
        return 2;       // failure.
} // Object_2D::NoParaDataAssociation() END ------------------------------------------------------------


// BRIEF Merges objects between two adjacent frames.
void Object_2D::MergeTwoFrameObj(Object_2D *ObjLastFrame)
{
    for (size_t m = 0; m < ObjLastFrame->Obj_c_MapPonits.size(); ++m)
    {
        bool bNewPoint = true;

        MapPoint *pMPLast = ObjLastFrame->Obj_c_MapPonits[m];
        cv::Mat PosLast = pMPLast->GetWorldPos();

        // whether a new points.
        for (size_t n = 0; n < this->Obj_c_MapPonits.size(); ++n)
        {
            MapPoint *pMPCurr = this->Obj_c_MapPonits[n];
            cv::Mat PosCurr = pMPCurr->GetWorldPos();

            if (cv::countNonZero(PosLast - PosCurr) == 0)
            {
                bNewPoint = false;
                break;
            }
        }

        if (bNewPoint)
        {
            this->Obj_c_MapPonits.push_back(pMPLast);

            this->ComputeMeanAndStandardFrame();
        }
    }
} // Object_2D::MergeTwoFrameObj(Object_2D* ObjLastFrame) END --------


// BRIEF compute the mean and standard deviation of objects in the map.
void Object_Map::ComputeMeanAndStandard()
{
    mSumPointsPos = 0;
    // remove bad points.
    {
        unique_lock<mutex> lock(mMutexMapPoints);
        vector<MapPoint *>::iterator pMP;
        int i = 0;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            i++;

            cv::Mat pos = (*pMP)->GetWorldPos();

            if ((*pMP)->isBad())
            {
                pMP = mvpMapObjectMappoints.erase(pMP);
            }
            else
            {
                mSumPointsPos += pos;
                ++pMP;
            }
        }
    }

    // mean(3d center).
    mCenter3D = mSumPointsPos / (mvpMapObjectMappoints.size());

    // step 1. standard deviation in 3 directions.
    float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    vector<float> x_pt, y_pt, z_pt;
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
    {
        cv::Mat pos = mvpMapObjectMappoints[i]->GetWorldPos();
        cv::Mat pos_ave = mCenter3D;

        // （x-x^）^2
        sum_x2 += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2 += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2 += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));

        x_pt.push_back(pos.at<float>(0));
        y_pt.push_back(pos.at<float>(1));
        z_pt.push_back(pos.at<float>(2));
    }
    mStandar_x = sqrt(sum_x2 / (mvpMapObjectMappoints.size()));
    mStandar_y = sqrt(sum_y2 / (mvpMapObjectMappoints.size()));
    mStandar_z = sqrt(sum_z2 / (mvpMapObjectMappoints.size()));

    if (x_pt.size() == 0)
        return;

    // step 2. standard deviation of centroids (observations from different frames).
    float sum_x2_c = 0, sum_y2_c = 0, sum_z2_c = 0;
    vector<float> x_c, y_c, z_c;
    for(size_t i = 0; i < this->mObjectFrame.size(); i++)
    {
        cv::Mat pos = this->mObjectFrame[i]->_Pos;
        cv::Mat pos_ave = this->mCenter3D;

        // （x-x^）^2
        sum_x2_c += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2_c += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2_c += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));
    }
    mCenterStandar_x = sqrt(sum_x2_c / (this->mObjectFrame.size()));
    mCenterStandar_y = sqrt(sum_y2_c / (this->mObjectFrame.size()));
    mCenterStandar_z = sqrt(sum_z2_c / (this->mObjectFrame.size()));

    // step 3. update object center and scale.
    if (this->mObjectFrame.size() < 5)
    {
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        sort(z_pt.begin(), z_pt.end());

        if ((x_pt.size() == 0) || (y_pt.size() == 0) || (z_pt.size() == 0))
        {
            this->bBadErase = true;
            return;
        }

        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];

        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        float z_min = z_pt[0];
        float z_max = z_pt[z_pt.size() - 1];

        // centre.
        mCuboid3D.cuboidCenter = Eigen::Vector3d((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);

        mCuboid3D.x_min = x_min;
        mCuboid3D.x_max = x_max;
        mCuboid3D.y_min = y_min;
        mCuboid3D.y_max = y_max;
        mCuboid3D.z_min = z_min;
        mCuboid3D.z_max = z_max;

        mCuboid3D.lenth = x_max - x_min;
        mCuboid3D.width = y_max - y_min;
        mCuboid3D.height = z_max - z_min;

        mCuboid3D.corner_1 = Eigen::Vector3d(x_min, y_min, z_min);
        mCuboid3D.corner_2 = Eigen::Vector3d(x_max, y_min, z_min);
        mCuboid3D.corner_3 = Eigen::Vector3d(x_max, y_max, z_min);
        mCuboid3D.corner_4 = Eigen::Vector3d(x_min, y_max, z_min);
        mCuboid3D.corner_5 = Eigen::Vector3d(x_min, y_min, z_max);
        mCuboid3D.corner_6 = Eigen::Vector3d(x_max, y_min, z_max);
        mCuboid3D.corner_7 = Eigen::Vector3d(x_max, y_max, z_max);
        mCuboid3D.corner_8 = Eigen::Vector3d(x_min, y_max, z_max);

        mCuboid3D.corner_1_w = Eigen::Vector3d(x_min, y_min, z_min);
        mCuboid3D.corner_2_w = Eigen::Vector3d(x_max, y_min, z_min);
        mCuboid3D.corner_3_w = Eigen::Vector3d(x_max, y_max, z_min);
        mCuboid3D.corner_4_w = Eigen::Vector3d(x_min, y_max, z_min);
        mCuboid3D.corner_5_w = Eigen::Vector3d(x_min, y_min, z_max);
        mCuboid3D.corner_6_w = Eigen::Vector3d(x_max, y_min, z_max);
        mCuboid3D.corner_7_w = Eigen::Vector3d(x_max, y_max, z_max);
        mCuboid3D.corner_8_w = Eigen::Vector3d(x_min, y_max, z_max);
    }

    // step 4. update object pose.
    UpdateObjPose();
    // world -> object frame.
    vector<float> x_pt_obj, y_pt_obj, z_pt_obj;
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
    {
        // world frame.
        Eigen::Vector3d PointPos_world = Converter::toVector3d(mvpMapObjectMappoints[i]->GetWorldPos());

        // object frame.
        Eigen::Vector3d PointPos_object = this->mCuboid3D.pose.inverse() * PointPos_world;
        x_pt_obj.push_back(PointPos_object[0]);
        y_pt_obj.push_back(PointPos_object[1]);
        z_pt_obj.push_back(PointPos_object[2]);
    }

    if (x_pt_obj.size() == 0)
        return;

    // rank.
    int s = x_pt_obj.size();
    sort(x_pt_obj.begin(), x_pt_obj.end());
    sort(y_pt_obj.begin(), y_pt_obj.end());
    sort(z_pt_obj.begin(), z_pt_obj.end());

    float x_min_obj = x_pt_obj[0];
    float x_max_obj = x_pt_obj[s - 1];
    float y_min_obj = y_pt_obj[0];
    float y_max_obj = y_pt_obj[s - 1];
    float z_min_obj = z_pt_obj[0];
    float z_max_obj = z_pt_obj[s - 1];

    // update object vertices and translate it to world frame.
    mCuboid3D.corner_1 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);

    // object frame -> world frame (without yaw, parallel to world frame).
    mCuboid3D.corner_1_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);

    // update scale and pose.
    this->mCuboid3D.lenth = x_max_obj - x_min_obj;
    this->mCuboid3D.width = y_max_obj - y_min_obj;
    this->mCuboid3D.height = z_max_obj - z_min_obj;
    this->mCuboid3D.cuboidCenter = (mCuboid3D.corner_2 + mCuboid3D.corner_8) / 2;
    UpdateObjPose();

    // maximum radius.
    float fRMax = 0.0;
    vector<cv::Mat> vCornerMat;
    vCornerMat.resize(8);
    for (int i = 0; i < 8; i++)
    {
        cv::Mat mDis = cv::Mat::zeros(3, 1, CV_32F);
        if (i == 0)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_1);
        if (i == 1)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_2);
        if (i == 2)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_3);
        if (i == 3)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_4);
        if (i == 4)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_5);
        if (i == 5)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_6);
        if (i == 6)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_7);
        if (i == 7)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_8);

        mDis = mCenter3D - vCornerMat[i];
        float fTmp = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) + mDis.at<float>(2) * mDis.at<float>(2));
        fRMax = max(fRMax, fTmp);
    }
    mCuboid3D.mfRMax = fRMax;

    // standard deviation of distance.
    float dis = 0;
    for (size_t i = 0; i < mObjectFrame.size(); i++)
    {
        float center_sum_x2 = 0, center_sum_y2 = 0, center_sum_z2 = 0;

        cv::Mat pos = mObjectFrame[i]->_Pos; 
        cv::Mat pos_ave = mCenter3D;

        center_sum_x2 = (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0)); // dis_x^2
        center_sum_y2 = (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1)); // dis_y^2
        center_sum_z2 = (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2)); // dis_z^2

        dis += sqrt(center_sum_x2 + center_sum_y2 + center_sum_z2);
    }
    mCenterStandar = sqrt(dis / (mObjectFrame.size()));
} // ComputeMeanAndStandard() END ---------------------------------------------------------------------------------------


// BRIEF remove outliers and refine the object position and scale by IsolationForest.
void Object_Map::IsolationForestDeleteOutliers()
{
    if(!biForest)
        return;

    if ((this->mnClass == 75) || (this->mnClass == 64) || (this->mnClass == 65))
        return;

    float th = 0.6;
    if (this->mnClass == 62)
        th = 0.65;

    std::mt19937 rng(12345);
    std::vector<std::array<float, 3>> data; // uint32_t

    if (mvpMapObjectMappoints.size() < 30)
        return;

    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[i];
        cv::Mat pos = pMP->GetWorldPos();

        std::array<float, 3> temp;
        temp[0] = pos.at<float>(0);
        temp[1] = pos.at<float>(1);
        temp[2] = pos.at<float>(2);
        data.push_back(temp);
    }

    // // STEP 1. generate some random 3D datapoints
    // for (uint32_t i = 0; i < 100; i++)
    // {
    // 	std::array<float, 3> temp;       // uint32_t
    // 	for (uint32_t j = 0; j < 3; j++)
    // 	{
    // 		temp[j] = iforest::UniformRandomNumber<float>::GenerateNext(rng, 40, 50);
    // 	}
    // 	data.push_back(temp);
    // }

    // // STEP 2. add a few anomalies
    // for (uint32_t i = 0; i < 10; i++)
    // {
    // 	std::array<float, 3> temp;       // uint32_t
    // 	for (uint32_t j = 0; j < 3; j++)
    // 	{
    // 		temp[j] = iforest::UniformRandomNumber<float>::GenerateNext(rng, 50, 100);
    // 	}
    // 	data.push_back(temp);
    // }

    iforest::IsolationForest<float, 3> forest; // uint32_t

    // STEP 3
    if (!forest.Build(50, 12345, data, ((int)mvpMapObjectMappoints.size() / 2)))
    {
        std::cerr << "Failed to build Isolation Forest.\n";
        return;
    }

    std::vector<double> anomaly_scores;

    // STEP 4
    if (!forest.GetAnomalyScores(data, anomaly_scores))
    {
        std::cerr << "Failed to calculate anomaly scores.\n";
        return;
    }

    std::vector<int> outliernum;
    for (uint32_t i = 0; i < (int)mvpMapObjectMappoints.size(); i++)
    {
        // std::cout << "Anomaly_score[" << i << "] " << anomaly_scores[i] << "\n";
        // std::cout << data[i][0] << ", " << data[i][1] << ", " << data[i][2] << std::endl;

        if (anomaly_scores[i] > th)
            outliernum.push_back(i);
    }

    if (outliernum.empty())
        return;

    // step 5. remove outliers.
    int numMappoint = -1;
    int numMOutpoint = 0;
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            numMappoint++;
            cv::Mat pos = (*pMP)->GetWorldPos();

            if (numMappoint == outliernum[numMOutpoint])
            {
                numMOutpoint++;
                pMP = mvpMapObjectMappoints.erase(pMP); 
                mSumPointsPos -= pos;
            }
            else
            {
                ++pMP;
            }
        }
    }
} // Object_Map::IsolationForestDeleteOutliers() END -----------------------------------------


// BRIEF whether associate successful.
bool Object_Map::DataAssociateUpdate(Object_2D *ObjectFrame,
                                     Frame &mCurrentFrame,
                                     cv::Mat &image,    // for debug.
                                     int Flag) // 1 Iou, 2 NP, 3 t-test，4 project. for debug.
{
    if (ObjectFrame->_class_id != mnClass)
        return false;

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // step 1. whether the box projected into the image changes greatly after the new point cloud is associated.
    if ((Flag != 1) && (Flag != 4))
    {
        cv::Rect ProjectRect1;
        cv::Rect ProjectRect2;

        // projected bounding box1.
        this->ComputeProjectRectFrame(image, mCurrentFrame);
        ProjectRect1 = this->mRectProject;

        // mixed points of frame object and map object.
        vector<float> x_pt;
        vector<float> y_pt;
        for (int i = 0; i < ObjectFrame->Obj_c_MapPonits.size(); ++i)
        {
            MapPoint *pMP = ObjectFrame->Obj_c_MapPonits[i];
            cv::Mat PointPosWorld = pMP->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }
        for (int j = 0; j < mvpMapObjectMappoints.size(); ++j)
        {
            MapPoint *pMP = mvpMapObjectMappoints[j];
            cv::Mat PointPosWorld = pMP->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        // rank.
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > image.cols)
            x_max = image.cols;
        if (y_max > image.rows)
            y_max = image.rows;

        // projected bounding box2. 
        ProjectRect2 = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

        // 4. 计算 Iou
        float fIou = Converter::bboxOverlapratio(ProjectRect1, ProjectRect2);
        float fIou2 = Converter::bboxOverlapratioFormer(ProjectRect2, ObjectFrame->mBoxRect);
        if ((fIou < 0.5) && (fIou2 < 0.8))
            return false;
    }

    // step 2. update the ID of the last frame
    if (mnLastAddID != (int)mCurrentFrame.mnId)
    {
        mnLastLastAddID = mnLastAddID;    
        mnLastAddID = mCurrentFrame.mnId;

        mLastLastRect = mLastRect;
        mLastRect = ObjectFrame->mBoxRect;

        mnConfidence++;

        ObjectFrame->current = true;

        mObjectFrame.push_back(ObjectFrame);
    }
    else
        return false;

    ObjectFrame->mnId = mnId;               
    ObjectFrame->mnWhichTime = mnConfidence;

    // step 3. Add the point cloud of the frame object to the map object
    for (size_t j = 0; j < ObjectFrame->Obj_c_MapPonits.size(); ++j)
    {
        MapPoint *pMP = ObjectFrame->Obj_c_MapPonits[j];

        cv::Mat pointPos = pMP->GetWorldPos();
        cv::Mat mDis = mCenter3D - pointPos;
        float fDis = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) + mDis.at<float>(2) * mDis.at<float>(2));

        float th = 1.0;
        if (mObjectFrame.size() > 5)
            th = 0.9;

        if (fDis > th * mCuboid3D.mfRMax)
            continue;

        if ((this->mObjectFrame.size() >= 10) && ((this->mnClass == 56) || (this->mnClass == 77)))
        {
            Eigen::Vector3d scale = this->mCuboid3D.pose.inverse() * Converter::toVector3d(pointPos);
            if ((abs(scale[0]) > 1.2 * this->mCuboid3D.lenth / 2) ||
                (abs(scale[1]) > 1.2 * this->mCuboid3D.width / 2) ||
                (abs(scale[2]) > 1.2 * this->mCuboid3D.height / 2))
                continue;
        }

        pMP->object_id = mnId;       
        pMP->object_class = mnClass; 

        // add points.
        map<int, int>::iterator sit;
        sit = pMP->object_id_vector.find(pMP->object_id);
        if (sit != pMP->object_id_vector.end())
        {
            int sit_sec = sit->second;                                           
            pMP->object_id_vector.erase(pMP->object_id);                         
            pMP->object_id_vector.insert(make_pair(pMP->object_id, sit_sec + 1));
        }
        else
        {
            pMP->object_id_vector.insert(make_pair(pMP->object_id, 1));
        }

        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = pMP->GetWorldPos();
                cv::Mat obj_map_pos = mvpMapObjectMappoints[m]->GetWorldPos();

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    mvpMapObjectMappoints[m]->have_feature = pMP->have_feature;
                    mvpMapObjectMappoints[m]->feature = pMP->feature;
                    new_point = false;

                    break;
                }
            }
            // new point.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);

                mvpMapCurrentNewMappoints.push_back(pMP);

                cv::Mat x3d = pMP->GetWorldPos();
                mSumPointsPos += x3d;
            }
        }
    }

    // step 4. the historical point cloud is projected into the image, and the points not in the box(should not on the edge) are removed.
    if ((ObjectFrame->mBox.x > 25) && (ObjectFrame->mBox.y > 25) &&
        (ObjectFrame->mBox.x + ObjectFrame->mBox.width < image.cols - 25) &&
        (ObjectFrame->mBox.y + ObjectFrame->mBox.height < image.rows - 25))
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            int sit_sec = 0;
            map<int , int>::iterator sit;
            sit = (*pMP)->object_id_vector.find(mnId);
            if (sit != (*pMP)->object_id_vector.end())
            {
                sit_sec = sit->second;
            }
            if (sit_sec > 8)
            {
                ++pMP;
                continue;
            }

            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            if ((u > 0 && u < image.cols) && (v > 0 && v < image.rows))
            {
                if (!ObjectFrame->mBoxRect.contains(cv::Point2f(u, v)))
                {
                    cv::circle(image, cv::Point2f(u, v), 3, cv::Scalar(0, 215, 256), -1);
                    pMP = mvpMapObjectMappoints.erase(pMP);
                    mSumPointsPos -= PointPosWorld;
                }
                else
                {
                    ++pMP;
                }
            }
            else
            {
                ++pMP;
            }
        }
    }

    // step 5. update object mean.
    this->ComputeMeanAndStandard();

    // step 6. i-Forest.
    this->IsolationForestDeleteOutliers();

    ObjectFrame->mAssMapObjCenter = this->mCenter3D;    
    mCurrentFrame.mvObjectFrame.push_back(ObjectFrame);

    return true;
} // Object_Map::DataAssociateUpdate() END ---------------------------------------------


// BRIEF projecting points to the image, constructing a bounding box.
void Object_Map::ComputeProjectRectFrame(cv::Mat &image, Frame &mCurrentFrame)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    vector<float> x_pt;
    vector<float> y_pt;
    for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[j];
        cv::Mat PointPosWorld = pMP->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        const float xc = PointPosCamera.at<float>(0);
        const float yc = PointPosCamera.at<float>(1);
        const float invzc = 1.0 / PointPosCamera.at<float>(2);

        float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
        float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

        x_pt.push_back(u);
        y_pt.push_back(v);

    }

    if (x_pt.size() == 0)
        return;

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max > image.cols)
        x_max = image.cols;
    if (y_max > image.rows)
        y_max = image.rows;

    mRectProject = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
} // ComputeProjectRectFrame() END ----------------------------------------------


// BRIEF whether merge two potantial associated objes.
void Object_Map::WhetherMergeTwoMapObjs(Map *mpMap)
{
    map<int, int>::iterator sit;
    for (sit = mReObj.begin(); sit != mReObj.end(); sit++)
    {
        int nObjId = sit->first;

        if (sit->second < 3)
            continue;

        if (mpMap->mvObjectMap[nObjId]->bBadErase)
            continue;

        // note: double t-test.
        bool bDoubelTtest = this->DoubleSampleTtest(mpMap->mvObjectMap[nObjId]);
        bool bSametime = true;

        // make sure they don't appear at the same time.
        map<int, int>::iterator sit2;
        sit2 = mmAppearSametime.find(nObjId);
        if (sit2 != mmAppearSametime.end())
        {
            continue;
        }
        else
            bSametime = false;

        if((!bSametime || bDoubelTtest))
        {
            int nAppearTimes1 = mObjectFrame.size();
            int nAppearTimes2 = mpMap->mvObjectMap[nObjId]->mObjectFrame.size();

            if (nAppearTimes1 > nAppearTimes2)
            {
                this->MergeTwoMapObjs(mpMap->mvObjectMap[nObjId]);
                this->ComputeMeanAndStandard();
                this->IsolationForestDeleteOutliers();
                mpMap->mvObjectMap[nObjId]->bBadErase = true;
            }
            else
            {
                mpMap->mvObjectMap[nObjId]->MergeTwoMapObjs(this);
                mpMap->mvObjectMap[nObjId]->ComputeMeanAndStandard();
                mpMap->mvObjectMap[nObjId]->IsolationForestDeleteOutliers();
                this->bBadErase = true;
            }
        }
    }
} // WhetherMergeTwoMapObjs(Map *mpMap) END --------------------------------------


// BRIEF double t-test.
bool Object_Map::DoubleSampleTtest(Object_Map *RepeatObj)
{
    // Read t-distribution boundary value.
    float tTestData[122][9] = {0};    
    ifstream infile;
    infile.open("./data/t_test.txt");
    for (int i = 0; i < 122; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            infile >> tTestData[i][j];
        }
    }
    infile.close();

    int ndf1 = this->mObjectFrame.size();
    float fMean1_x = this->mCenter3D.at<float>(0, 0);
    float fMean1_y = this->mCenter3D.at<float>(1, 0);
    float fMean1_z = this->mCenter3D.at<float>(2, 0);
    float fCenterStandar1_x = this->mCenterStandar_x;
    float fCenterStandar1_y = this->mCenterStandar_y;
    float fCenterStandar1_z = this->mCenterStandar_z;

    int ndf2 = RepeatObj->mObjectFrame.size();
    float fMean2_x = RepeatObj->mCenter3D.at<float>(0, 0);
    float fMean2_y = RepeatObj->mCenter3D.at<float>(1, 0);
    float fMean2_z = RepeatObj->mCenter3D.at<float>(2, 0);
    float fCenterStandar2_x = RepeatObj->mCenterStandar_x;
    float fCenterStandar2_y = RepeatObj->mCenterStandar_y;
    float fCenterStandar2_z = RepeatObj->mCenterStandar_z;

    // Combined standard deviation.
    float d_x = sqrt( ( ( (ndf1-1)*fMean1_x*fMean1_x + (ndf2-1)*fMean2_x*fMean2_x ) / (ndf1 + ndf2 - 2) ) *
                      (1/ndf1 + 1/ndf2) );
    float d_y = sqrt( ( ( (ndf1-1)*fMean1_y*fMean1_y + (ndf2-1)*fMean2_y*fMean2_y ) / (ndf1 + ndf2 - 2) ) *
                      (1/ndf1 + 1/ndf2) );
    float d_z = sqrt( ( ( (ndf1-1)*fMean1_z*fMean1_z + (ndf2-1)*fMean2_z*fMean2_z ) / (ndf1 + ndf2 - 2) ) *
                      (1/ndf1 + 1/ndf2) );

    // t-test
    float t_test_x = ( fMean1_x -fMean2_x ) / d_x;
    float t_test_y = ( fMean1_y -fMean2_y ) / d_y;
    float t_test_z = ( fMean1_z -fMean2_z ) / d_z;

    // Satisfy t test in 3 directions.
    if ((t_test_x < tTestData[min((ndf1 + ndf2 - 2), 121)][5]) &&
        (t_test_y < tTestData[min((ndf1 + ndf2 - 2), 121)][5]) &&
        (t_test_z < tTestData[min((ndf1 + ndf2 - 2), 121)][5]))
    {
        return true;
    }
    else
        return false;
} // DoubleSampleTtest() END -----------------------------------------------------------------------------


// BRIEF merge two objects.
void Object_Map::MergeTwoMapObjs(Object_Map *RepeatObj)
{
    // step 1. update points.
    for (int i = 0; i < RepeatObj->mvpMapObjectMappoints.size(); i++)
    {
        MapPoint *pMP = RepeatObj->mvpMapObjectMappoints[i];

        cv::Mat pointPos = pMP->GetWorldPos();
        Eigen::Vector3d scale = this->mCuboid3D.pose.inverse() * Converter::toVector3d(pointPos);
        if ((abs(scale[0]) > 1.1 * this->mCuboid3D.lenth / 2) ||
            (abs(scale[1]) > 1.1 * this->mCuboid3D.width / 2) ||
            (abs(scale[2]) > 1.1 * this->mCuboid3D.height / 2))
        {
            continue;
        }

        pMP->object_id = mnId;       
        pMP->object_class = mnClass; 

        map<int, int>::iterator sit;
        sit = pMP->object_id_vector.find(pMP->object_id);
        if (sit != pMP->object_id_vector.end()) 
        {
            int sit_sec = sit->second;                                            
            pMP->object_id_vector.erase(pMP->object_id);                          
            pMP->object_id_vector.insert(make_pair(pMP->object_id, sit_sec + 1)); 
        }
        else
        {
            pMP->object_id_vector.insert(make_pair(pMP->object_id, 1));
        }
        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = pMP->GetWorldPos();
                cv::Mat obj_map_pos = mvpMapObjectMappoints[m]->GetWorldPos();

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    mvpMapObjectMappoints[m]->have_feature = pMP->have_feature;
                    mvpMapObjectMappoints[m]->feature = pMP->feature;
                    new_point = false;

                    break;
                }
            }
            // new points.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);

                cv::Mat x3d = pMP->GetWorldPos();
                mSumPointsPos += x3d;
            }
        }
    }

    // step 2. update frame objects.
    for (int j = 0; j < RepeatObj->mObjectFrame.size(); j++)
    {
        Object_2D *ObjectFrame = RepeatObj->mObjectFrame[j];

        ObjectFrame->mnId = mnId;
        mnConfidence++;

        mObjectFrame.push_back(ObjectFrame);
    }

    // step 3. update the co-view relationship
    {
        map<int, int>::iterator sit;
        for (sit = RepeatObj->mmAppearSametime.begin(); sit != RepeatObj->mmAppearSametime.end(); sit++)
        {
            int nObjId = sit->first;
            int sit_sec = sit->second;

            map<int, int>::iterator sit2;
            sit2 = mmAppearSametime.find(nObjId);
            if (sit2 != mmAppearSametime.end())
            {
                int sit_sec2 = sit2->second;
                mmAppearSametime.erase(nObjId);
                mmAppearSametime.insert(make_pair(nObjId, sit_sec2 + sit_sec));
            }
            else
                mmAppearSametime.insert(make_pair(nObjId, 1));
        }
    }

    // step 4. update the last observed frame.
    int nOriginLastAddID = mnLastAddID;
    int nOriginLastLatsAddID = mnLastLastAddID;
    cv::Rect OriginLastRect = mLastRect;
    // this object appeared recently.
    if (mnLastAddID > RepeatObj->mnLastAddID)
    {
        if (nOriginLastLatsAddID > RepeatObj->mnLastAddID)
        {
        }
        else
        {
            mnLastLastAddID = RepeatObj->mnLastAddID;
            mLastLastRect = RepeatObj->mObjectFrame[RepeatObj->mObjectFrame.size() - 1]->mBoxRect;
        }
    }
    // RepeatObj appeared recently.
    else
    {
        mnLastAddID = RepeatObj->mnLastAddID;
        mLastRect = RepeatObj->mObjectFrame[RepeatObj->mObjectFrame.size() - 1]->mBoxRect;

        if (nOriginLastAddID > RepeatObj->mnLastLastAddID)
        {
            mnLastLastAddID = nOriginLastAddID;
            mLastLastRect = OriginLastRect;
        }
        else
        {
            mnLastLastAddID = RepeatObj->mnLastLastAddID;
            mLastLastRect = RepeatObj->mObjectFrame[RepeatObj->mObjectFrame.size() - 2]->mBoxRect;
        }
    }

    // step 5. update direction.
    if (((mnClass == 73) || (mnClass == 64) || (mnClass == 65) 
        || (mnClass == 66) || (mnClass == 56)))
    {
        if(RepeatObj->mvAngleTimesAndScore.size() > 0)
        {
            for (auto &row_repeat : RepeatObj->mvAngleTimesAndScore)
            {
                bool new_measure = true;

                if(this->mvAngleTimesAndScore.size() > 0)
                {
                    for (auto &row_this : this->mvAngleTimesAndScore)
                    {
                        if(row_repeat[0] == row_this[0])
                        {
                            row_this[1] += row_repeat[1];

                            row_this[2] = row_this[2] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[2] * (row_repeat[1] / row_this[1]);
                            
                            row_this[3] = row_this[3] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[3] * (row_repeat[1] / row_this[1]);
                            
                            row_this[4] = row_this[4] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[4] * (row_repeat[1] / row_this[1]);

                            new_measure = false;
                            break;
                        }
                    }
                }

                if(new_measure == true)
                {
                    this->mvAngleTimesAndScore.push_back(row_repeat);
                }
            }
        }

        if(this->mvAngleTimesAndScore.size() > 0)
        {
            int best_num = 0;
            float best_score = 0.0;
            for(int i = 0; i < min(6, (int)this->mvAngleTimesAndScore.size()); i++)
            {
                float fScore = this->mvAngleTimesAndScore[i][2];
                if(fScore > best_score)
                {
                    best_score = fScore;
                    best_num = i;
                }
            }

            this->mCuboid3D.rotY = this->mvAngleTimesAndScore[best_num][0];
            this->mCuboid3D.mfErrorParallel = this->mvAngleTimesAndScore[best_num][3];
            this->mCuboid3D.mfErroeYaw = this->mvAngleTimesAndScore[best_num][4];
            this->UpdateObjPose();
        }
    }
} // MergeTwoMapObjs() END -----------------------------------------------------------


// BRIEF check whether two objects overlap.
bool Object_Map::WhetherOverlap(Object_Map *CompareObj)
{
    // distance between two centers.
    float dis_x = abs(mCuboid3D.cuboidCenter(0) - CompareObj->mCuboid3D.cuboidCenter(0));
    float dis_y = abs(mCuboid3D.cuboidCenter(1) - CompareObj->mCuboid3D.cuboidCenter(1));
    float dis_z = abs(mCuboid3D.cuboidCenter(2) - CompareObj->mCuboid3D.cuboidCenter(2));

    float sum_lenth_half = mCuboid3D.lenth / 2 + CompareObj->mCuboid3D.lenth / 2;
    float sum_width_half = mCuboid3D.width / 2 + CompareObj->mCuboid3D.width / 2;
    float sum_height_half = mCuboid3D.height / 2 + CompareObj->mCuboid3D.height / 2;

    // whether overlap.
    if ((dis_x < sum_lenth_half) && (dis_y < sum_width_half) && (dis_z < sum_height_half))
        return true;
    else
        return false;
} // WhetherOverlap() END ----------------------------------------------------------------


// BRIEF big one gets smaller, the smaller one stays the same. (No significant effect! Almost no use.)
void Object_Map::BigToSmall(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z)
{
    // in which direction does the overlap occur.
    bool bxMin = false;
    bool bxMax = false;
    bool byMin = false;
    bool byMax = false;
    bool bzMin = false;
    bool bzMax = false;

    // x_min
    if ((SmallObj->mCuboid3D.x_min > this->mCuboid3D.x_min) && (SmallObj->mCuboid3D.x_min < this->mCuboid3D.x_max))
        bxMin = true;
    // x_max
    if ((SmallObj->mCuboid3D.x_max > this->mCuboid3D.x_min) && (SmallObj->mCuboid3D.x_max < this->mCuboid3D.x_max))
        bxMax = true;
    // y_min
    if ((SmallObj->mCuboid3D.y_min > this->mCuboid3D.y_min) && (SmallObj->mCuboid3D.y_min < this->mCuboid3D.y_max))
        byMin = true;
    // y_max
    if ((SmallObj->mCuboid3D.y_max > this->mCuboid3D.y_min) && (SmallObj->mCuboid3D.y_max < this->mCuboid3D.y_max))
        byMax = true;
    // z_min
    if ((SmallObj->mCuboid3D.z_min > this->mCuboid3D.z_min) && (SmallObj->mCuboid3D.z_min < this->mCuboid3D.z_max))
        bzMin = true;
    // z_max
    if ((SmallObj->mCuboid3D.z_max > this->mCuboid3D.z_min) && (SmallObj->mCuboid3D.z_max < this->mCuboid3D.z_max))
        bzMax = true;

    // false: one direction，ture: two directions.
    bool bx = false;
    bool by = false;
    bool bz = false;
    // x 
    if ((bxMin = true) && (bxMax = true))
        bx = true;
    else
        bx = false;
    // y 
    if ((byMin = true) && (byMax = true))
        by = true;
    else
        by = false;
    // z 
    if ((bzMin = true) && (bzMax = true))
        bz = true;
    else
        bz = false;

    // Which direction to eliminate?
    int nFlag; // 0:x   1:y   2:z   3: surround

    // x
    if ((bx == false) && (by == true) && (bz == true))
        nFlag = 0;
    // y
    if ((bx == true) && (by == false) && (bz == true))
        nFlag = 1;
    // z
    if ((bx == true) && (by == true) && (bz == false))
        nFlag = 2;

    if ((bx == false) && (by == false) && (bz == true))
    {
        if (min(overlap_x, overlap_y) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_y) == overlap_y)
            nFlag = 1;
    }

    if ((bx == false) && (by == true) && (bz == false))
    {
        if (min(overlap_x, overlap_z) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_z) == overlap_z)
            nFlag = 2;
    }

    if ((bx == true) && (by == false) && (bz == false))
    {
        if (min(overlap_y, overlap_z) == overlap_y)
            nFlag = 1;
        else if (min(overlap_y, overlap_z) == overlap_z)
            nFlag = 2;
    }

    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.

        // remove points in the overlap volume.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();

            // points in the smaller object.
            if ((PointPosWorld.at<float>(0) > SmallObj->mCuboid3D.x_min) && (PointPosWorld.at<float>(0) < SmallObj->mCuboid3D.x_max) &&
                (PointPosWorld.at<float>(1) > SmallObj->mCuboid3D.y_min) && (PointPosWorld.at<float>(1) < SmallObj->mCuboid3D.y_max) &&
                (PointPosWorld.at<float>(2) > SmallObj->mCuboid3D.z_min) && (PointPosWorld.at<float>(2) < SmallObj->mCuboid3D.z_max))
                pMP = mvpMapObjectMappoints.erase(pMP);
            else
            {
                ++pMP;
            }
        }
    }

    this->ComputeMeanAndStandard();
} // BigToSmall() END -----------------------------------------------------------------------------------------------------------------


// BRIEF Divide the overlap area of two objects equally.  (No significant effect! Almost no use.)
void Object_Map::DivideEquallyTwoObjs(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z)
{
    {
        unique_lock<mutex> lock(mMutexMapPoints);

        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();

            if (((PointPosWorld.at<float>(0) > AnotherObj->mCuboid3D.cuboidCenter(0) - (AnotherObj->mCuboid3D.lenth / 2 - overlap_x / 2)) &&
                 (PointPosWorld.at<float>(0) < AnotherObj->mCuboid3D.cuboidCenter(0) + (AnotherObj->mCuboid3D.lenth / 2 - overlap_x / 2))) &&
                ((PointPosWorld.at<float>(1) > AnotherObj->mCuboid3D.cuboidCenter(1) - (AnotherObj->mCuboid3D.width / 2 - overlap_y / 2)) &&
                 (PointPosWorld.at<float>(1) < AnotherObj->mCuboid3D.cuboidCenter(1) + (AnotherObj->mCuboid3D.width / 2 - overlap_y / 2))) &&
                ((PointPosWorld.at<float>(2) > AnotherObj->mCuboid3D.cuboidCenter(2) - (AnotherObj->mCuboid3D.height / 2 - overlap_z / 2)) &&
                 (PointPosWorld.at<float>(2) < AnotherObj->mCuboid3D.cuboidCenter(2) + (AnotherObj->mCuboid3D.height / 2 - overlap_z / 2))))
            {
                pMP = mvpMapObjectMappoints.erase(pMP);
            }

            else
            {
                ++pMP;
            }
        }
    }
} // DivideEquallyTwoObjs() END --------------------------------------------------------------------------------


// BRIEF Dealing with two overlapping objects.
void Object_Map::DealTwoOverlapObjs(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z)
{
    bool bIou = false;       // false: Iou is large.
    bool bVolume = false;    // false: small volume difference.
    bool bSame_time = false; // false: doesn't simultaneous appearance.
    bool bClass = false;     // false: different classes.

    float fThis_obj_volume = (mCuboid3D.lenth * mCuboid3D.width) * mCuboid3D.height;
    float fOverlap_obj_volume = (OverlapObj->mCuboid3D.lenth * OverlapObj->mCuboid3D.width) * OverlapObj->mCuboid3D.height;

    // compute Iou.
    float overlap_volume = (overlap_x * overlap_y) * overlap_z;
    if ((overlap_volume / (fThis_obj_volume + fOverlap_obj_volume - overlap_volume)) >= 0.3)
        bIou = true;
    else
        bIou = false;

    // compute the volume difference.
    if ((fThis_obj_volume > 2 * fOverlap_obj_volume) || (fOverlap_obj_volume > 2 * fThis_obj_volume))
        bVolume = true;
    else
        bVolume = false;

    // whether simultaneous appearance.
    map<int, int>::iterator sit;
    sit = mmAppearSametime.find(OverlapObj->mnId);
    if (sit != mmAppearSametime.end())
    {
        if (sit->second > 3)
            bSame_time = true;
        else
            bSame_time = false;
    }
    else
        bSame_time = false;

    // class.
    if (mnClass == OverlapObj->mnClass)
        bClass = true;
    else
        bClass = false;

    // case 1: IOU is large, the volume difference is small, doesn't simultaneous appearance, same class --> the same object, merge them.
    if ((bIou == true) && (bVolume == false) && (bSame_time == false) && (bClass == true))
    {
        if (this->mObjectFrame.size() >= OverlapObj->mObjectFrame.size())
        {
            this->MergeTwoMapObjs(OverlapObj);
            OverlapObj->bBadErase = true;
        }
        else
        {
            OverlapObj->MergeTwoMapObjs(this);
            this->bBadErase = true;
        }
    }

    // case 2: may be a false detection.
    else if ((bVolume == true) && (bSame_time == false) && (bClass == true))
    {
        if ((this->mObjectFrame.size() >= OverlapObj->mObjectFrame.size()) && (fThis_obj_volume > fOverlap_obj_volume))
            OverlapObj->bBadErase = true;
        else if ((this->mObjectFrame.size() < OverlapObj->mObjectFrame.size()) && (fThis_obj_volume < fOverlap_obj_volume))
            this->bBadErase = true;
    }

    // case 3: divide the overlap area of two objects equally.  (No significant effect.)
    else if ((bIou == true) && (bVolume == false) && (bSame_time == true) && (bClass == true))
    {
        this->DivideEquallyTwoObjs(OverlapObj, overlap_x, overlap_y, overlap_z);
        OverlapObj->DivideEquallyTwoObjs(OverlapObj, overlap_x, overlap_y, overlap_z);

        this->ComputeMeanAndStandard();
        OverlapObj->ComputeMeanAndStandard();
    }

    // case 4: big one gets smaller, the smaller one stays the same. (No significant effect.)
    else if ((bIou == false) && (bVolume == true) && (bSame_time == true) && (bClass == false))
    {
        if (fThis_obj_volume > fOverlap_obj_volume)
            this->BigToSmall(OverlapObj, overlap_x, overlap_y, overlap_z);
        else if (fThis_obj_volume < fOverlap_obj_volume)
            OverlapObj->BigToSmall(this, overlap_x, overlap_y, overlap_z);
    }

    // case 5: 
    else if((bIou == true) && (bSame_time == false) && (bClass == true))
    {
        if (this->mObjectFrame.size()/2 >= OverlapObj->mObjectFrame.size())
        {
            this->MergeTwoMapObjs(OverlapObj);
            OverlapObj->bBadErase = true;
        }
        else if(OverlapObj->mObjectFrame.size()/2 >= this->mObjectFrame.size())
        {
            OverlapObj->MergeTwoMapObjs(this);
            this->bBadErase = true;
        }
    }

    // TODO case ...... and so on ......
} // DealTwoOverlapObjs() END ---------------------------------------------------------------


// // BRIEF update object scale (for optimization， not used in this version)
// void Object_Map::UpdateObjScale(Eigen::Vector3d Scale)
// {
//     unique_lock<mutex> lock(mMutex);

//     this->mCuboid3D.lenth = Scale[0];
//     this->mCuboid3D.width = Scale[1];
//     this->mCuboid3D.height = Scale[2];
// }


// BRIEF update object pose.
void Object_Map::UpdateObjPose()
{
    unique_lock<mutex> lock(mMutex);

    // Rotation matrix.
    float cp = cos(mCuboid3D.rotP);
    float sp = sin(mCuboid3D.rotP);
    float sr = sin(mCuboid3D.rotR);
    float cr = cos(mCuboid3D.rotR);
    float sy = sin(mCuboid3D.rotY);
    float cy = cos(mCuboid3D.rotY);
    Eigen::Matrix<double, 3, 3> REigen;
    REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    cv::Mat Ryaw = Converter::toCvMat(REigen);

    // Transformation matrix.
    cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Twobj.rowRange(0, 3).col(3);
    cv::Mat result = Rcw * Ryaw;

    Twobj.at<float>(0, 0) = result.at<float>(0, 0);
    Twobj.at<float>(0, 1) = result.at<float>(0, 1);
    Twobj.at<float>(0, 2) = result.at<float>(0, 2);
    //Twobj.at<float>(0, 3) = mCenter3D.at<float>(0);
    Twobj.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];
    Twobj.at<float>(1, 0) = result.at<float>(1, 0);
    Twobj.at<float>(1, 1) = result.at<float>(1, 1);
    Twobj.at<float>(1, 2) = result.at<float>(1, 2);
    //Twobj.at<float>(1, 3) = mCenter3D.at<float>(1);
    Twobj.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];
    Twobj.at<float>(2, 0) = result.at<float>(2, 0);
    Twobj.at<float>(2, 1) = result.at<float>(2, 1);
    Twobj.at<float>(2, 2) = result.at<float>(2, 2);
    //Twobj.at<float>(2, 3) = mCenter3D.at<float>(2);
    Twobj.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];
    Twobj.at<float>(3, 0) = 0;
    Twobj.at<float>(3, 1) = 0;
    Twobj.at<float>(3, 2) = 0;
    Twobj.at<float>(3, 3) = 1;

    // note no yaw.
    cv::Mat Twobj_without_yaw = cv::Mat::eye(4, 4, CV_32F);
    Twobj_without_yaw.at<float>(0, 3) = mCenter3D.at<float>(0);
    Twobj_without_yaw.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];
    Twobj_without_yaw.at<float>(2, 3) = mCenter3D.at<float>(2);

    // SE3.
    g2o::SE3Quat obj_pose = Converter::toSE3Quat(Twobj);
    g2o::SE3Quat obj_pose_without_yaw = Converter::toSE3Quat(Twobj_without_yaw);

    this->mCuboid3D.pose = obj_pose;
    this->mCuboid3D.pose_without_yaw = obj_pose_without_yaw;
} // UpdateObjScale()

} // namespace ORB_SLAM2