#include "TouchManager.h"
#include "WindowUtils.h"

void TouchManager::setupWindow() {
	removeWindowBorder();

	ofPoint target(-PROJW, 0);
	ofSetWindowPosition(target.x, target.y);
	ofSetWindowShape(PROJW + DISPW, max(DISPH, PROJH));

	/* Adjust position to compensate for the effects of the borderless adjustment */
	ofPoint actualPos(ofGetWindowPositionX(), ofGetWindowPositionY());
	ofPoint adjTarget = target - (actualPos - target);
	ofSetWindowPosition(adjTarget.x, adjTarget.y);
}

void TouchManager::threadedFunction()
{
	while (isThreadRunning()) {
		ofShortPixels test1 = visionDeviceManager->getDepthShortPixels();
		ofShortPixels test2 = visionDeviceManager->getIrShortPixels();

		bgthread->updateFrame(visionDeviceManager->isDepthFrameNew(), visionDeviceManager->getDepthShortPixels());
		touchTracker->updateFrame(visionDeviceManager->getDepthShortPixels(), visionDeviceManager->getIrShortPixels(), visionDeviceManager->isDepthFrameNew());
	}
}

void TouchManager::setup()
{
	ofSetFrameRate(60);
	setupWindow();
	// kinect setup
	depthWidth = visionDeviceManager->getDepthWidth();
	depthHeight = visionDeviceManager->getDepthHeight();
	isDepthFrameNew = visionDeviceManager->isDepthFrameNew();

	bgthread = new BackgroundUpdaterThread(depthWidth, depthHeight, isDepthFrameNew, visionDeviceManager->getDepthShortPixels());
	bgthread->startThread();

	touchTracker = new IRDepthTouchTracker(depthWidth, depthHeight, visionDeviceManager->getDepthShortPixels(), visionDeviceManager->getIrShortPixels(), *bgthread, isDepthFrameNew);
	touchTracker->startThread();
	setupDebug();
}

void TouchManager::setupDebug()
{
	depthviz.allocate(depthWidth, depthHeight, OF_IMAGE_GRAYSCALE);
}

void TouchManager::update()
{
	vector<FingerTouch> newTouches;
	if (touchTracker->update(newTouches)) {
		handleTouches(newTouches);
	}
	updateDebug();
}

void TouchManager::handleTouches(const vector<FingerTouch>& newTouches)
{
	map<int, FingerTouch> newTouchMap;
	set<int> touchDown, touchUp;
	map<int, FingerTouch> allTouches;

	/* Sort touches */
	for (auto &i : touchMap) {
		allTouches[i.first] = i.second;
	}

	for (auto touch : newTouches) {
		newTouchMap[touch.id] = touch;
		allTouches[touch.id] = touch;

		if (!touchMap.count(touch.id) || (!touchMap[touch.id].touched && touch.touched))
			touchDown.insert(touch.id);
	}

	for (auto &i : touchMap) {
		if (!newTouchMap.count(i.first) || (i.second.touched && !newTouchMap[i.first].touched))
			touchUp.insert(i.first);
	}

	touchMap = newTouchMap;
}

void TouchManager::updateDebug()
{
	/* Check if the frame is actually new */
	//	uint64_t curDepthTimestamp = depthStream.getFrameTimestamp();
	if (visionDeviceManager->isDepthFrameNew())
		return;
	//	lastDepthTimestamp = curDepthTimestamp;

	/* Debugging */
	auto &depthPixels = visionDeviceManager->getDepthShortPixels();
	uint16_t *depthpx = depthPixels.getPixels();
	const int dw = depthPixels.getWidth();
	const int dh = depthPixels.getHeight();

	uint8_t *depthvizpx = depthviz.getPixelsRef().getPixels();

	/* Convert depth data for visualization purposes */
	for (int i = 0; i<dw*dh; i++) {
		depthvizpx[i] = depthpx[i];
	}

	depthviz.update();
}

void TouchManager::drawProjector()
{
	/* In this function, draw points in real-world coordinates (metres) */
	ofSetLineWidth(0.002);

	/* Reproject touches */
	for (auto &entry : touchMap) {
		auto &touch = entry.second;
		ofPoint worldPt = getBackgroundWorldPoint(touch.tip);
		if (touch.touched) {
			ofNoFill();
			ofSetColor(0, 255, 0);
		}
		else {
			ofNoFill();
			ofSetColor(255, 0, 0);
		}
		ofCircle(worldPt, 0.010);
		ofDrawBitmapString(ofVAArgsToString("%.2f\n%d", touch.touchZ, touch.id), worldPt);
	}
}

void TouchManager::drawDebug()
{
	const int dw = visionDeviceManager->getDepthWidth();
	const int dh = visionDeviceManager->getDepthHeight();

	// depthviz.draw(0, 0);	// depth 영상
	drawText("Depth", 0, 0, HAlign::left, VAlign::top);

	//bgthread->drawDebug(0, dh);   //background 영상
	touchTracker->drawDebug(0, -dh);	// diff, diff+edge, edge, blob 영상
	// color영상 draw
	this->colorTouchDraw(300,300);

	drawText(ofVAArgsToString("FPS: %.1f\n", ofGetFrameRate())
		+ ofVAArgsToString("BG Update FPS: %.1f\n", bgthread->fps.fps)
		+ ofVAArgsToString("Touch Update FPS: %.1f\n", touchTracker->fps.fps), DISPW, 0, HAlign::right, VAlign::top);

	/*
	int debugMouseX = mouseX - PROJW;
	int debugMouseY = mouseY;
	if (0 <= debugMouseX && debugMouseX < dw && 0 <= debugMouseY && debugMouseY < dh) {
	ofVec2f pos(debugMouseX, debugMouseY);

	string description;
	ofPoint curPt = getLiveWorldPoint(pos);
	description += ofVAArgsToString("curpos: %.6f, %.6f, %.6f\n", curPt.x, curPt.y, curPt.z);

	ofPoint bgPt = getBackgroundWorldPoint(pos);
	description += ofVAArgsToString("bgpos:  %.6f, %.6f, %.6f\n", bgPt.x, bgPt.y, bgPt.z);

	drawText(description, 0, DISPH, HAlign::left, VAlign::bottom);
	}
	*/
}

void TouchManager::teardown()
{
	delete touchTracker;
	delete bgthread;
}

void TouchManager::colorTouchDraw(int x, int y)
{
	const int dw = visionDeviceManager->getDepthWidth();
	const int dh = visionDeviceManager->getDepthHeight();

	map<int, FingerTouch>::iterator iter;
	if (touchMap.size() > 0) {
		vector<ofPoint> visited;
		visited.clear();
		ofPoint pt1 = touchMap.begin()->second.tip;
		ofDrawCircle(pt1.x+x, pt1.y+y, 10);
		visited.push_back(pt1);

		
		for (iter = touchMap.begin(); iter != touchMap.end(); iter++) {
			ofPoint pt = iter->second.tip;
			bool isNew = true;
			for(int i=0; i<visited.size(); ++i){
				pt1 = visited.at(i);
				ofPoint distpt = pt1 - pt;
				float dist = sqrt(distpt.x*distpt.x + distpt.y*distpt.y);
				if (dist < 50) {
					isNew = false;
				}

			}
			if (isNew == true) {
				visited.push_back(pt);
				ofDrawCircle(pt.x+x, pt.y+y, 10);
			}
			//		drawText("point" + ofToString(pt.x) + " " + ofToString(pt.y), pt.x * 2, pt.y * 2, HAlign::left, VAlign::top);
		}
		
	}
	ofImage colorImg = visionDeviceManager->getColorImage();
	colorImg.draw(x, y, dw, dh);

}

ofPoint TouchManager::getWorldPoint(const ofVec2f &depthPos, bool live) {
	int x0 = floor(depthPos.x);
	int y0 = floor(depthPos.y);
	if (x0 < 0 || x0 >= depthWidth - 1 || y0 < 0 || y0 >= depthHeight - 1)
		return ofPoint(0, 0, 0);

	/* Linearly interpolate the world point */
	ofPoint ret;
	for (int x = x0; x <= x0 + 1; x++) {
		for (int y = y0; y <= y0 + 1; y++) {
			DepthSpacePoint dpt = { x, y };

			int depth=0;
			int index = (int)dpt.Y * depthWidth + (int)dpt.X;
			if (live) {
				depth = visionDeviceManager->getDepthShortPixels()[index];
//					depthStream.getShortPixelsRef().getPixels()[index]; // current (finger) depth
			}
			else {
				depth = bgthread->getBackgroundMean().getPixels()[index]; // stable (background) depth
			}

			float weight = (1 - fabsf(depthPos.x - x)) * (1 - fabsf(depthPos.y - y));

			/* Map depth point to camera point */
			CameraSpacePoint wpt;
			HRESULT hr;
			bool hrbool;
			ofVec2f dpt_temp = { dpt.X, dpt.Y };
			ofVec3f wpt_temp;
			hrbool = visionDeviceManager->mapDepthToCameraHResult(dpt_temp, depth, &wpt_temp);
			if (!hrbool) {
				ofLogError() << "MapDepthPointToCameraSpace failed";
				return ofPoint(0, 0, 0);
			}

			ret += weight * ofPoint(wpt.X, wpt.Y, wpt.Z);
		}
	}

	return ret;
}