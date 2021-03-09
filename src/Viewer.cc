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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer( System* pSystem, FrameDrawer *pFrameDrawer, 
                MapDrawer *pMapDrawer, Tracking *pTracking, 
                const string &strSettingPath, const string &flag):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false), mflag(flag)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    mfx = fSettings["Camera.fx"];
    mfy = fSettings["Camera.fy"];
    mcx = fSettings["Camera.cx"];
    mcy = fSettings["Camera.cy"];

}

void Viewer::Run()
{
    mbFinished = false;

    // pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);
    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1200, 900);   // 1920,1080.
    // pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",mImageWidth+175,mImageHeight);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowCamera("menu.Show Camera",true,true);           
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuShowSemiDense("menu.Show SemiDense",true,true);
    pangolin::Var<double> menuSigmaTH("menu.Sigma",0.02,1e-10,0.05,false);
    pangolin::Var<bool> menuCameraView("menu.Camera View",true,true);
    pangolin::Var<bool> menuShowModel("menu.Show Model", false,true);
    pangolin::Var<bool> menuShowTexture("menu.Show Texture", false,true);

    pangolin::Var<bool> menuShowCubeObj("menu.Show CubeObj",true,true);
    pangolin::Var<bool> menuShowQuadricObj("menu.Show QuadricObj",true,true);

    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    pangolin::Var<bool> menuShowBottle("menu.Show Bottles",true,true);
    pangolin::Var<bool> menuShowChair("menu.Show Chairs",true,true);
    pangolin::Var<bool> menuShowTvmonitors("menu.Show Tvmonitors",true,true);
    pangolin::Var<bool> menuShowKeyboard("menu.Show Keyboard",true,true);
    pangolin::Var<bool> menuShowMouse("menu.Show Mouse",true,true);
    pangolin::Var<bool> menuShowBook("menu.Show Books",true,true);
    pangolin::Var<bool> menuShowBear("menu.Show Bear",true,true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
//                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
//                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
            // carv: using calibrated camera center and focal length
            pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000),
            pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -mImageWidth/mImageHeight)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("Raw Image");
    cv::moveWindow("Raw Image", 40, 20);
    cv::namedWindow("Point, Line and Object Detection");
    cv::moveWindow("Point, Line and Object Detection", 40, 360);
    cv::namedWindow("Quadric Projection");
    cv::moveWindow("Quadric Projection", 40, 710);

    bool bFollow = true;
    bool bLocalizationMode = false;

    // carv: camera close up view
    bool bCameraView = true;
    pangolin::OpenGlMatrix projectionAbove = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mViewpointF,mViewpointF,
                                                                        mImageWidth/2,mImageHeight/2,0.1,1000);
    pangolin::OpenGlMatrix projectionCamera = pangolin::ProjectionMatrix(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.1,1000);
    pangolin::OpenGlMatrix viewAbove = pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0);
    pangolin::OpenGlMatrix viewCamera = pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0.0,-1.0, 0.0);


    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
//            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        // carv: setup viewpoint to see model
        if(menuCameraView && !bCameraView)
        {
            s_cam.SetProjectionMatrix(projectionCamera);
            s_cam.SetModelViewMatrix(viewCamera);
            bCameraView = true;
        }
        else if(!menuCameraView && bCameraView)
        {
            s_cam.SetProjectionMatrix(projectionAbove);
            s_cam.SetModelViewMatrix(viewAbove);
            bCameraView = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        if(menuShowCamera)
            mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();
        if(menuShowSemiDense)
            mpMapDrawer->DrawSemiDense(menuSigmaTH);

        // step draw objects.
        if(menuShowCubeObj || menuShowQuadricObj)
        {
            mpMapDrawer->DrawObject(menuShowCubeObj, menuShowQuadricObj,
                                    mflag,
                                    menuShowBottle, menuShowChair, menuShowTvmonitors,
                                    menuShowKeyboard,menuShowMouse,menuShowBook,menuShowBear);
        }

        // TODO check opengl error?.
        //CheckGlDieOnError()
        // step carv: show model or triangle with light from camera
        if(menuShowModel && menuShowTexture) {
            mpMapDrawer->DrawModel();
        }
        else if (menuShowModel && !menuShowTexture) {
            mpMapDrawer->DrawTriangles(Twc);
        }
        else if (!menuShowModel && menuShowTexture) {
            mpMapDrawer->DrawFrame();
        }
        //CheckGlDieOnError()


        pangolin::FinishFrame();

        // gray image.
        cv::Mat im = mpFrameDrawer->DrawFrame();
        if(!im.empty())
        {
            cv::Mat resizeimg;
            cv::resize(im, resizeimg, cv::Size(640*0.7, 480*0.7), 0, 0, cv::INTER_CUBIC);
            cv::imshow("Point, Line and Object Detection", resizeimg);
        }

        // color image.
        cv::Mat RawImage = mpFrameDrawer->GetRawColorImage();
        if(!RawImage.empty())
        {
            cv::Mat resizeimg;
            cv::resize(RawImage, resizeimg, cv::Size(640*0.7, 480*0.7), 0, 0, cv::INTER_CUBIC);
            cv::imshow("Raw Image", resizeimg);
        }

        // quadric image.
        cv::Mat QuadricImage = mpFrameDrawer->GetQuadricImage();
        if(!QuadricImage.empty())
        {
            cv::Mat resizeimg;
            cv::resize(QuadricImage, resizeimg, cv::Size(640*0.7, 480*0.7), 0, 0, cv::INTER_CUBIC);
            cv::imshow("Quadric Projection", resizeimg);
        }

        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;

            menuShowSemiDense = true;
            menuCameraView = true;
            menuShowModel = true;
            menuShowTexture = true;

            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
