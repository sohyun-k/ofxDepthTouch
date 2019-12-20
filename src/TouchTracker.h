/* Generic parent class for touch tracking implementations. */

#pragma once

#include "ofMain.h"
#include "ofxKinect2.h"
#include "ofxOpenCv.h"

#include "FPSTracker.h"
#include "BackgroundUpdaterThread.h"
#include "Touch.h"

class TouchTracker : public ofThread {
	/* Forbid copying */

protected:
//	TouchTracker &operator=(const TouchTracker &);
//	TouchTracker(const TouchTracker &);

protected:
	const int w, h;
//	ofxKinect2::DepthStream &depthStream;
//	ofxKinect2::IrStream &irStream;
	ofShortPixels& irShortPixels;
	ofShortPixels& depthShortPixels;
	BackgroundUpdaterThread &background;

	ofMutex touchLock;
	bool touchesUpdated;
	vector<FingerTouch> touches;
	int nextTouchId;

public:
	FPSTracker fps;


	/* Public methods */
/*	TouchTracker(ofxKinect2::DepthStream &depthStream, ofxKinect2::IrStream &irStream, BackgroundUpdaterThread &background)
        : w(depthStream.getWidth()), h(depthStream.getHeight()), depthStream(depthStream), irStream(irStream), background(background) {
		touchesUpdated = false;
		nextTouchId = 1;
    }
*/
	TouchTracker(int width, int height, ofShortPixels& depthShortPixels, ofShortPixels& irShortPixels, BackgroundUpdaterThread &background, bool isNewFrame)
		: w(width), h(height), depthShortPixels(depthShortPixels), irShortPixels(irShortPixels), background(background){
		touchesUpdated = false;
		nextTouchId = 1;
	}
	/* The responsibility of stopping the thread is in the subclass: it must be the first thing the destructor does. */
	virtual ~TouchTracker() {}

	virtual void drawDebug(float x, float y) {}
	virtual bool update(vector<FingerTouch> &retTouches)= 0;
	virtual void updateFrame(ofShortPixels& depthShortPixels, ofShortPixels& irShortPixels, bool isNewFrame) =0;

};
