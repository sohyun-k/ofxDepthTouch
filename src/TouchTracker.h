/* Generic parent class for touch tracking implementations. */

#pragma once

#include "ofMain.h"
#include "ofxKinect2.h"
#include "ofxOpenCv.h"

#include "FPSTracker.h"
#include "BackgroundUpdaterThread.h"
#include "Touch.h"

class TouchTracker : public ofThread {
private:
	/* Forbid copying */
	TouchTracker &operator=(const TouchTracker &);
	TouchTracker(const TouchTracker &);

protected:
	int w, h;
//	ofxKinect2::DepthStream &depthStream;
	ofShortPixels& depthPix;
//	ofxKinect2::IrStream &irStream;
	ofShortPixels& irPix;

	BackgroundUpdaterThread &background;

	bool isNew;

	ofMutex touchLock;
	bool touchesUpdated;
	vector<FingerTouch> touches;
	int nextTouchId;

public:
	FPSTracker fps;

	/* Public methods */
	TouchTracker(ofShortPixels& depthPix, ofShortPixels& irPix, BackgroundUpdaterThread &background, int w, int h, bool isNew)
    :depthPix(depthPix), irPix(irPix), background(background), isNew(isNew){
		this->w = w;
		this->h = h;
		touchesUpdated = false;
		nextTouchId = 1;
    }

	/* The responsibility of stopping the thread is in the subclass: it must be the first thing the destructor does. */
	virtual ~TouchTracker() {}

	virtual void drawDebug(float x, float y) {}
	virtual bool update(vector<FingerTouch> &retTouches) = 0;
};
