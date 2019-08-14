#pragma once

#ifndef OFX_DEPTHTOUCH_H
#define OFX_DEPTHTOUCH_H

#include "ofMain.h"
#include "Touch.h"
#include "BackgroundUpdaterThread.h"

class ofxDepthTouch {
	ofxDepthTouch(ofShortPixels& pix, int w, int h);

	class BackgroundUpdaterThread *bgthread;
	class TouchTracker *touchTracker;

	int depthWidth, depthHeight;
	ofShortPixels& depthPix;
	HRESULT hr;
	
	map<int, FingerTouch> touchMap;

	ofPoint getWorldPoint(const ofVec2f &depthPt, bool live);
	ofPoint getBackgroundWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, false); }
	ofPoint getLiveWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, true); }

	void setupWindow();

	void teardown();
	void setDepthWidth(int w);
	void setDepthHeight(int h);
	void setDepthPix(ofShortPixels& pix); // depthStream.getShortPixelsRef().getPixels()
	void setHR(HRESULT hr); // hr = kinect->getMapper()->MapDepthPointToCameraSpace(dpt, depth, &wpt);


};

#endif