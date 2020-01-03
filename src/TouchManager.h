#pragma once
#include "VisionDeviceManager.hpp"
#include "Touch.h"
#include "BackgroundUpdaterThread.h"
#include "IRDepthTouchTracker.h"
#include "TextUtils.h"
#include "geomConfig.h"

class TouchManager : public ofThread {
private:
	VisionDeviceManager::Ptr visionDeviceManager;
	class BackgroundUpdaterThread *bgthread;
	class TouchTracker *touchTracker;

	int depthWidth, depthHeight;
	bool isDepthFrameNew;
	bool isTouchActivated = false;
	ARVboMesh::Ptr touch_mesh = make_shared<ARVboMesh>();
	ofMatrix4x4 system_pose;
	vector<ofPoint> previousPoint;
	vector<ofPoint> touchPoint;
	vector<ofPoint> touch3DPoint;

public:
	map<int, FingerTouch> touchMap;
	ofImage depthviz;

private:
	ofPoint getWorldPoint(const ofVec2f &depthPt, bool live);
	ofPoint getBackgroundWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, false); }
	ofPoint getLiveWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, true); }

	void setupWindow();
	void makeTouchMesh(ofMatrix4x4 _system_pose);

protected:
	void threadedFunction();

public:

	void init(VisionDeviceManager::Ptr _visionDeviceManager)
	{
		visionDeviceManager = _visionDeviceManager;
	}

	void setup();
	void setupDebug();
	void update(ofMatrix4x4 _system_pose = ofMatrix4x4());
	void handleTouches(const vector<FingerTouch> &newTouches);
	void updateDebug();
	void drawProjector();
	void drawDebug();

	void teardown();
	void colorTouchDraw(int x, int y, int w, int h);
	void meshDrawDebug();

	bool getIsTouchActivate();
	void setIsTouchActivate(bool isTouchActivated);

	ARVboMesh::Ptr getTouchMesh() {
		return touch_mesh;
	}
};