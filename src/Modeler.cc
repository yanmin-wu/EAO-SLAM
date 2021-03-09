
#include <mutex>
#include "Modeler.h"
#include "Map.h"
#include "KeyFrame.h"

#include "LineDetector.h"


// Model Class
Model::Model(const vector<dlovi::Matrix> &modelPoints, const list<dlovi::Matrix> &modelTris)
{
    mbNotErase = false;
    mbToBeErased = false;
    mData.first = modelPoints;
    mData.second = modelTris;
}

vector<dlovi::Matrix> & Model::GetPoints()
{
    return mData.first;
}

list<dlovi::Matrix> & Model::GetTris()
{
    return mData.second;
}

void Model::SetNotErase()
{
    unique_lock<mutex> lock(mMutexErase);
    mbNotErase = true;
}

void Model::SetErase()
{
	{
    	unique_lock<mutex> lock(mMutexErase);
    	mbNotErase = false;
	}

    if (mbToBeErased){
        Release();
    }
}

void Model::Release()
{
	{
    	unique_lock<mutex> lock(mMutexErase);
    	if (mbNotErase){
        	mbToBeErased = true;
        	return;
    	}
	}
    // suicide, need to be careful
    // delete this;

    // deallocate memory
    vector<dlovi::Matrix>().swap(mData.first);
    list<dlovi::Matrix>().swap(mData.second);
}


// Modeler Class
Modeler::Modeler(ORB_SLAM2::Map* pMap)
{
    mnLastNumLines = 2;
    mbFirstKeyFrame = true;

    mpMap = pMap;
    mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
    mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptToProcessRef());
    mAlgInterface.rewind();
}

void Modeler::WriteModel(std::string filename)
{
    mAlgInterface.writeCurrentModelToFile(filename);
}

void Modeler::AddKeyFrameEntry(ORB_SLAM2::KeyFrame *pKF)
{
    if (mbFirstKeyFrame) {
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addFirstKeyFrameInsertionEntry(pKF);
        mbFirstKeyFrame = false;
    } else {
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addKeyFrameInsertionEntry(pKF);
    }

}

void Modeler::AddLineSegmentKeyFrameEntry(ORB_SLAM2::KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexTranscript);
    mTranscriptInterface.addLineSegmentKeyFrameInsertionEntry(pKF);
}

void Modeler::RunRemainder()
{
    std::cout << "running transcript" << std::endl;

    mAlgInterface.runRemainder();
}

void Modeler::RunOnce()
{
    std::cout << "running transcript once" << std::endl;

    mAlgInterface.setComputingModel(false);
    mAlgInterface.runRemainder();
    mAlgInterface.setComputingModel(true);
    mAlgInterface.computeCurrentModel();
}

void Modeler::UpdateModel()
{
    std::cout << "updating model" << std::endl; 

    std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
    Model* pNewModel = new Model(objModel.first, objModel.second);

    mpMap->UpdateModel(pNewModel);
}

bool Modeler::CheckNewTranscriptEntry()
{
    unique_lock<mutex> lock(mMutexTranscript);
    int numLines = mTranscriptInterface.getTranscriptRef()->numLines();
    if (numLines > mnLastNumLines) {
        mnLastNumLines = numLines;
        mTranscriptInterface.UpdateTranscriptToProcess();
        std::cout << "checking transcript: " << numLines << std::endl;
        return true;
    } else {
        return false;
    }
}