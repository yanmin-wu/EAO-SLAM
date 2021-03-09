/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* 
* Modification: EAO-SLAM
* Version: 1.0
* Created: 07/18/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include "Modeler.h"

#include <mutex>

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class Object_Map;

class Map
{
public:
    Map();

    void SetModeler(Modeler *pModeler);
    Modeler* GetModeler();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    // void AddObjectMapPoints(MapPoint* pMP);

    void UpdateModel(Model* pModel);
    Model* GetModel();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    std::vector<Object_Map*> GetObjects();
    std::vector<cv::Mat> GetCubeCenters();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    
    vector<Object_Map*> mvObjectMap;    // objects in the map.
    vector<cv::Mat> cube_center;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    std::set<MapPoint*> mvpObjectMapPoints;

    long unsigned int mnMaxKFid;

    Model* mpModel;

    Modeler* mpModeler;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
