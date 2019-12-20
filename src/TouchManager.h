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


public:
	map<int, FingerTouch> touchMap;
	ofImage depthviz;

private:
	ofPoint getWorldPoint(const ofVec2f &depthPt, bool live);
	ofPoint getBackgroundWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, false); }
	ofPoint getLiveWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, true); }

	void setupWindow();

protected:
	void threadedFunction();

public:

	void init(VisionDeviceManager::Ptr _visionDeviceManager)
	{
		visionDeviceManager = _visionDeviceManager;
	}

	void setup();
	void setupDebug();
	void update();
	void handleTouches(const vector<FingerTouch> &newTouches);
	void updateDebug();
	void drawProjector();
	void drawDebug();

	void teardown();
	void colorTouchDraw(int x, int y, int w, int h);
};