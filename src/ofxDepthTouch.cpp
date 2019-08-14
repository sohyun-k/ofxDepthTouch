#include "ofxDepthTouch.h"
#include "WindowUtils.h"
#include "geomConfig.h"

ofxDepthTouch::ofxDepthTouch(ofShortPixels& pix, int w, int h)
:depthPix(pix) {
	this->depthWidth = w;
	this->depthHeight = h;
}

ofPoint ofxDepthTouch::getWorldPoint(const ofVec2f & depthPt, bool live)
{
	int x0 = floor(depthPt.x);
	int y0 = floor(depthPt.y);
	if (x0 < 0 || x0 >= depthWidth - 1 || y0 < 0 || y0 >= depthHeight - 1)
		return ofPoint(0, 0, 0);

	/* Linearly interpolate the world point */
	ofPoint ret;
	for (int x = x0; x <= x0 + 1; x++) {
		for (int y = y0; y <= y0 + 1; y++) {
			ofVec2f dpt = { x, y };
			
			int depth;
			int index = (int)dpt.y * depthWidth + (int)dpt.x;
			if (live) {
				depth = depthPix.getPixels()[index]; // current (finger) depth
			}
			else {
				depth = bgthread->getBackgroundMean().getPixels()[index]; // stable (background) depth
			}

			float weight = (1 - fabsf(depthPt.x - x)) * (1 - fabsf(depthPt.y - y));

			/* Map depth point to camera point */
			ofPoint wpt;
			HRESULT hr;
			if (!SUCCEEDED(hr)) {
				ofLogError() << "MapDepthPointToCameraSpace failed";
				return ofPoint(0, 0, 0);
			}

			ret += weight * ofPoint(wpt.x, wpt.y, wpt.z);
		}
	}

	return ret;
}

void ofxDepthTouch::setupWindow()
{
	removeWindowBorder();

	ofPoint target(-PROJW, 0);
	ofSetWindowPosition(target.x, target.y);
	ofSetWindowShape(PROJW + DISPW, max(DISPH, PROJH));

	/* Adjust position to compensate for the effects of the borderless adjustment */
	ofPoint actualPos(ofGetWindowPositionX(), ofGetWindowPositionY());
	ofPoint adjTarget = target - (actualPos - target);
	ofSetWindowPosition(adjTarget.x, adjTarget.y);
}

void ofxDepthTouch::teardown()
{
	delete bgthread;
}

void ofxDepthTouch::setDepthWidth(int w)
{
	depthWidth = w;
}

void ofxDepthTouch::setDepthHeight(int h)
{
	depthHeight = h;
}

void ofxDepthTouch::setDepthPix(ofShortPixels& pix)
{
	depthPix = pix;
}

void ofxDepthTouch::setHR(HRESULT hr)
{
	this->hr = hr;
}
