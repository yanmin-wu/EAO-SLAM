/*
 * =============================================
 *      Filename:  Object.h
 *
 *      Description:
 *      Version: 1.0
 *      Created: 09/19/2019
 *      Author: Yanmin Wu
 *      E-mail: wuyanminmax@gmail.com
 * ==============================================
 */

#ifndef OBJECT_H
#define OBJECT_H

#include "System.h"
#include "bitset"
#include "MapPoint.h"
#include <mutex>

// cube slam.
#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float,5,1> Vector5f;

namespace ORB_SLAM2
{
    class Frame;
    class MapPoint;
    class KeyFrame;
    class Map;
    
    // BRIEF the object in current frame.
    class Object_2D
    {
        public:
            int _class_id;      // class id.
            float mScore;       // Probability.

            float left;         // size.
            float right;
            float top;
            float bottom;
            float mWidth;
            float mHeight;

            cv::Rect mBoxRect;              // cv::Rect format.
            cv::Rect mRectFeaturePoints;    // the bounding box constructed by object feature points.
            BoxSE mBox;                     // BoxSE
            cv::Point2f box_center_2d;      // 2D center.

            vector< MapPoint*>  Obj_c_MapPonits;        // object points in current frame.
            cv::Mat _Pos;                               // current object center (3d, world).
            cv::Mat mAssMapObjCenter;                   // map object center.
            float mStandar_x, mStandar_y, mStandar_z;   // standard deviation
            int mCountMappoint;                         // = Obj_c_MapPonits.size().

            int mnId;               // object ID.
            int mnWhichTime;

            int LastAddId;                  
            cv::Point2f point_center_2d;    
            bool mbHaveCube = false;        
            // cuboid* mDetectedCube;          // cube slam.

            bool few_mappoint;      
            bool bOnEdge;           // on the edge of the image.

            bool First_obj;
            int confidence;
            int add_id;
            bool bad = false;
            bool current = false;

            int nMayRepeat = 0;                 
            std::map<int, float> mReIdAndIou;   // potential objects.

            vector< MapPoint*>  Obj_k_MapPonits;  // not used. 
            vector< MapPoint*>  co_MapPonits;     // not used.      
            // vector< MapPoint*>  pro_MapPonits;       
            // vector<cv::Mat> pro_MapPoints_camera;    

            cv::Mat sum_pos_3d;         // Summation of points observed in the current frame.
            cv::Mat sum_pos_3d_map;     // Summation of points observed in the map.

            // line.
            Eigen::MatrixXd mObjLinesEigen; 

            void CopyBoxes(const BoxSE &box);           // copy box to object_2d.
            void ComputeMeanAndStandardFrame();         // compute the mean and standard deviation of object points in current frame.
            void RemoveOutliersByBoxPlot(Frame &mCurrentFrame); // remove outliers by boxplot.
            void ObjectDataAssociation(Map* mpMap, Frame &mCurrentFrame, cv::Mat &image, string &flag);    // data association.
            int  NoParaDataAssociation(Object_Map* ObjectMapSingle, Frame &mCurrentFrame, cv::Mat &image); // NP.
            void MergeTwoFrameObj(Object_2D* ObjLastFrame);
        
        protected:
            std::mutex mMutexFrameObjMapPoints;
    };

    // brief 
    struct Cuboid3D
    {
        //     7------6
        //    /|     /|
        //   / |    / |
        //  4------5  |
        //  |  3---|--2
        //  | /    | /
        //  0------1
        // lenth ：corner_2[0] - corner_1[0]
        // width ：corner_2[1] - corner_3[1]
        // height：corner_2[2] - corner_6[2]

        // 8 vertices.
        Eigen::Vector3d corner_1;
        Eigen::Vector3d corner_2;
        Eigen::Vector3d corner_3;
        Eigen::Vector3d corner_4;
        Eigen::Vector3d corner_5;
        Eigen::Vector3d corner_6;
        Eigen::Vector3d corner_7;
        Eigen::Vector3d corner_8;

        // 8 vertices (without rotation).
        Eigen::Vector3d corner_1_w;
        Eigen::Vector3d corner_2_w;
        Eigen::Vector3d corner_3_w;
        Eigen::Vector3d corner_4_w;
        Eigen::Vector3d corner_5_w;
        Eigen::Vector3d corner_6_w;
        Eigen::Vector3d corner_7_w;
        Eigen::Vector3d corner_8_w;

        float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.
        Eigen::Vector3d cuboidCenter;       // the center of the Cube, not the center of mass of the object

        float lenth;
        float width;
        float height;

        g2o::SE3Quat pose;                      // 6 dof pose.
        g2o::SE3Quat pose_without_yaw;          // 6 dof pose without rotation.

        // angle.
        float rotY = 0.0;        
        float rotP = 0.0;
        float rotR = 0.0;

        float mfRMax;

        // line.
        float mfErrorParallel;
        float mfErroeYaw;
    };

    // BRIEF 3d object in the map.
    class Object_Map
    {
        public:
            std::vector<Object_2D*> mObjectFrame;
            cv::Rect mLastRect;
            cv::Rect mLastLastRect;
            cv::Rect mPredictRect;
            cv::Rect mRectProject;
            int mnId;
            int mnClass;
            int mnConfidence;
            bool mbFirstObserve;
            int mnAddedID;      
            int mnLastAddID;
            int mnLastLastAddID;
            std::set<int> msFrameId;
            vector< MapPoint*> mvpMapObjectMappoints;
            vector< MapPoint*> mvpMapCurrentNewMappoints;

            cv::Mat mSumPointsPos;
            cv::Mat mCenter3D;          
            float mStandar_x, mStandar_y, mStandar_z;
            float mCenterStandar_x, mCenterStandar_y, mCenterStandar_z;
            float mCenterStandar; 

            int nMayRepeat = 0;                 // maybe a repeat object.
            std::map<int, int> mReObj;          // potential associated objects.
            std::map<int, int> mmAppearSametime;// object id and times simultaneous appearances .

            bool bBadErase = false;

            Cuboid3D mCuboid3D;                  // cuboid.
            vector<cv::Mat> mvPointsEllipsoid;   // not used.

            std::vector<Vector5f> mvAngleTimesAndScore;    // Score of sampling angle.

            void ComputeMeanAndStandard();
            void IsolationForestDeleteOutliers();
            bool DataAssociateUpdate(   Object_2D* ObjectFrame, 
                                        Frame &mCurrentFrame, 
                                        cv::Mat &image,
                                        int Flag);

            void ComputeProjectRectFrame(cv::Mat &image, Frame &mCurrentFrame); 
            void WhetherMergeTwoMapObjs(Map *mpMap);
            void MergeTwoMapObjs(Object_Map *RepeatObj);
            bool DoubleSampleTtest(Object_Map *RepeatObj);
            void DealTwoOverlapObjs(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
            bool WhetherOverlap(Object_Map *CompareObj);
            void BigToSmall(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);          
            void DivideEquallyTwoObjs(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);

            // void UpdateObjScale(Eigen::Vector3d Scale);    // for optimization.
            void UpdateObjPose();      // update object pose.

        protected:
            std::mutex mMutexMapPoints;
            std::mutex mMutex;
    };
}
#endif //OBJECT_H