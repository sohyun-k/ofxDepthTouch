//
//  BackgroundUpdaterThread.h
//  Dynamic background updater thread.
//
//  Created by Robert Xiao on February 20, 2015.
//
//

#pragma once

#include "ofMain.h"
#include "ofxKinect2.h"
#include "FPSTracker.h"

struct bgPixelState;


class BackgroundUpdaterThread : public ofThread {
private:
	int width, height;
	bgPixelState *bgpixels;
	ofFloatPixels bgmean, bgstdev;
//	ofxKinect2::DepthStream &depthStream;
	ofShortPixels& depthPix;
	bool isFrameNew;
	int curFrame;

	/* Debugging */
	ofImage backgroundStateDebug;

	/* Forbid copying */
	BackgroundUpdaterThread &operator=(const BackgroundUpdaterThread &);
	BackgroundUpdaterThread(const BackgroundUpdaterThread &);
protected:
	void threadedFunction();
public:
	FPSTracker fps;

	/* Public methods */
//	BackgroundUpdaterThread(ofxKinect2::DepthStream &depthStream);
	BackgroundUpdaterThread(ofShortPixels& depthPix, int w, int h, bool isNew);
	virtual ~BackgroundUpdaterThread();

	void setDynamicUpdate(bool dynamic);
	void captureBackground();

	void drawDebug(float x, float y);
	void update();
	const ofFloatPixels &getBackgroundMean() const { return bgmean; }
	const ofFloatPixels &getBackgroundStdev() const { return bgstdev; }
};
