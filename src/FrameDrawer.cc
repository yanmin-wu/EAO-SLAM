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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    if(state == Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vCurrentKeys.size(); i++)
        {
            cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(0,0,255),-1);
        }

        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        for(int i=0;i<N;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    bool bInBox = false;

                    if(have_detected)
                    {
                        // NOTE [EAO-SLAM] points in the bounding box.
                        for (auto&box : Dboxes)
                        {
                            int left = box.x;
                            int right = box.x+box.width;
                            int top = box.y;
                            int bottom = box.y+box.height;

                            if((vCurrentKeys[i].pt.x > left)&&(vCurrentKeys[i].pt.x < right)
                                &&(vCurrentKeys[i].pt.y > top)&&(vCurrentKeys[i].pt.y < bottom)) 
                            {
                                cv::circle(im, vCurrentKeys[i].pt, 2, colors[box.m_class%4], -1);

                                bInBox = true;
                                break;
                            }
                        }
                    }

                    if(bInBox == false)
                    {
                        // cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,255),-1);
                    }
                    
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    // yolo detection.
    DrawYoloInfo(im, false);

    //return imWithInfo;
    return im;
}

// BRIEF [EAO-SLAM] draw yolo image.
cv::Mat FrameDrawer::DrawYoloFrame()
{
    cv::Mat imRGB;
    mRGBIm.copyTo(imRGB);
    DrawYoloInfo(imRGB, true);
    return imRGB;
}

// BRIEF [EAO-SLAM] get color image.
cv::Mat FrameDrawer::GetRawColorImage()
{
    cv::Mat imRGB;
    mRGBIm.copyTo(imRGB);
    return imRGB;
}

// BRIEF [EAO-SLAM] draw quadric image.
cv::Mat FrameDrawer::GetQuadricImage()
{
    cv::Mat imRGB;
    mQuadricIm.copyTo(imRGB);
    return imRGB;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

// BRIEF draw yolo text.
cv::Mat FrameDrawer::DrawYoloInfo(cv::Mat &im, bool bText)
{
    for (auto&box : Dboxes) 
    {
        if(bText)
        {
            cv::putText(im,                             
                        class_names[box.m_class],       
                        box.tl(),                       
                        cv::FONT_HERSHEY_DUPLEX  ,      
                        1.0,                            
                        colors[box.m_class%4],          
                        // cv::Scalar(0,255,0), 
                        2);                             
        }

        // draw lines in the box
        for(int obj_id = 0; obj_id < DObjsLines.size(); obj_id ++)
        {
            for(int line_id = 0; line_id < DObjsLines[obj_id].rows(); line_id++)
            {
                cv::Scalar lineColor;
                int R = ( rand() % (int) ( 255 + 1 ) );
                int G = ( rand() % (int) ( 255 + 1 ) );
                int B = ( rand() % (int) ( 255 + 1 ) );
                lineColor = cv::Scalar( R, G, B );

                cv::line(   im, 
                            cv::Point2f( DObjsLines[obj_id](line_id, 0), DObjsLines[obj_id](line_id, 1)), 
                            cv::Point2f( DObjsLines[obj_id](line_id, 2), DObjsLines[obj_id](line_id, 3)), 
                            cv::Scalar( 255, 255, 0 ), 
                            //lineColor,
                            2.0);
            }
        }

        // draw bounding box.
        cv::rectangle(  im,
                        box,
                        colors[box.m_class%4],
                        2);
    }

    return im;
} // DrawYoloInfo().

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    pTracker->mCurrentFrame.mColorImage.copyTo(mRGBIm);         // [EAO] copy color image.
    pTracker->mCurrentFrame.mQuadricImage.copyTo(mQuadricIm);   // [EAO] copy quadric image.

    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    // NOTE [EAO]
    // key line
    Dkeylines_raw = pTracker->mCurrentFrame.keylines_raw;
    Dkeylines_out = pTracker->mCurrentFrame.keylines_out;
    DTimeStamp = pTracker->mCurrentFrame.mTimeStamp;
    // object detection results.
    Dboxes = pTracker->mCurrentFrame.boxes;
    have_detected = pTracker->mCurrentFrame.have_detected;
    // object line.
    DObjsLines = pTracker->mCurrentFrame.vObjsLines;

    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
