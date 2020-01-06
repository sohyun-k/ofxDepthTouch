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

void TouchManager::makeTouchMesh()
{
	touch_mesh->clear();
	touch_mesh->setMode(OF_PRIMITIVE_POINTS);
	touch_mesh->disableIndices();
	touch_mesh->enableColors();
	// calibrationMeta 안함
	for (int i = 0; i < touch3DPoint.size(); ++i) {
		touch_mesh->addVertex(touch3DPoint.at(i));
		touch_mesh->addColor(ofColor::hotPink);
	}

}

void TouchManager::makeTouchPoint(ofMatrix4x4 _system_pose)
{
	system_pose = _system_pose;
	previousPoint = touchPoint;
	previous3DPoint = touch3DPoint;
	touchPoint.clear();
	touch3DPoint.clear();

	vector<ofPoint> allTouches;

	vector<vector<ofPoint>> touch_buff;

	if (touchMap.size() > 0) {
		map<int, FingerTouch>::iterator iter;

		for (iter = touchMap.begin(); iter != touchMap.end(); iter++) {
			if (!iter->second.touched) continue;

			ofPoint pt = iter->second.tip;
			//			vector<ofPoint> first;
			//			first.push_back(pt);
			//			touch_buff.push_back(first);

			allTouches.push_back(pt);
			bool isNew = true;
			for (int i = 0; i < touch_buff.size(); ++i) {
				for (int j = 0; j < touch_buff.at(i).size(); ++j) {
					ofPoint pt1 = touch_buff.at(i).at(j);
					ofPoint distpt = pt1 - pt;
					float dist = sqrt(distpt.x*distpt.x + distpt.y*distpt.y);
					if (dist < 40 && dist > 0) { //50 픽셀 이하면 같은 점으로 취급함
						isNew = false;
						touch_buff.at(i).push_back(pt);
						//						cout << "dist : " << dist << endl;
						break;
					}
					else if (dist == 0) continue;
					if (!isNew) break;
				}
			}
			if (isNew == true) {
				vector<ofPoint> temp;
				temp.push_back(pt);
				touch_buff.push_back(temp);
			}
		}

		for (int i = 0; i < touch_buff.size(); ++i) {
			if (touch_buff.at(i).size() == 1) {
				touchPoint.push_back(touch_buff.at(i).at(0));
			}
			else {
				// 중심점으로 찾기
				/*
				ofPoint tot = ofPoint(0,0,0);
				for (int j = 0; j < touch_buff.at(i).size(); ++j) {
					tot += touch_buff.at(i).at(j);

					int size = touch_buff.at(i).size();
					tot = ofPoint(tot.x / size, tot.y / size, tot.z / size);
					touchPoint.push_back(tot);
				}
				*/
				//임의의 한점 선택
				///*
				int mid = touch_buff.at(i).size() / 2;
				touchPoint.push_back(touch_buff.at(i).at(0));
				//*/
				//이전 포인트와 가장 거리가 짧은 포인트
				/*
				float dist = 999999;
				ofPoint shortPt = touch_buff.at(i).at(0);
				for (int j = 0; j < touch_buff.at(i).size(); ++j) {
					for (int idx = 0; idx < previousPoint.size(); ++idx) {
						ofPoint distPt = previousPoint.at(idx) - touch_buff.at(i).at(j);
						float newdist = sqrt(distPt.x*distPt.x + distPt.y*distPt.y);
						if (newdist < dist) {
							dist = newdist;
							shortPt = touch_buff.at(i).at(j);
						}
					}
				}
				touchPoint.push_back(shortPt);
				*/
			}
		}
		if (touchPoint.size() == previousPoint.size()) {
			if (touchPoint.size() == 1) {
				ofPoint currPt = touchPoint.at(0);
				ofPoint prePt = previousPoint.at(0);
				ofPoint distpt = currPt - prePt;
				//				cout << "튕길까??" << endl;
				float dist = sqrt(distpt.x*distpt.x + distpt.y*distpt.y);
				if (dist > 30) {
					touchPoint = previousPoint;
					cout << "튕겼다!" << endl;
				}
			}
		}
		//3개이상의 touchPoint일 경우 이전 Point
		if (touchPoint.size() > 2) {
			touchPoint = previousPoint;
		}

		//touch Point의 3D 좌표를 구함
		touch3DPoint.clear();
		touch3DPoint = visionDeviceManager->depthToCamera(touchPoint);
		for (int idx = 0; idx < touch3DPoint.size(); ++idx) {
			touch3DPoint.at(idx) = touch3DPoint.at(idx) * ofPoint(-1, 1, -1) * system_pose;
		}

		allTouches = visionDeviceManager->depthToCamera(allTouches);
		// touchMesh에 저장

		
		//		for (int i = 0; i < allTouches.size(); ++i) {
		//			touch_mesh->addVertex(allTouches.at(i) * ofPoint(-1, 1, -1) * system_pose);
		//			touch_mesh->addColor(ofColor::green);
		//		}
		//		cout << allTouches.size() << endl;
	}
	else {
		static auto notTouchedTime = ofGetElapsedTimeMillis();
		auto now = ofGetElapsedTimeMillis();
		if (now - notTouchedTime > 100) {
			previousPoint.clear();
		}
		else {
			touchPoint = previousPoint;
			touch3DPoint = previous3DPoint;
			allTouches = visionDeviceManager->depthToCamera(allTouches);
		}
	}
}

void TouchManager::threadedFunction()
{
	while (isThreadRunning()) {
		bgthread->updateFrame(visionDeviceManager->isDepthFrameNew(), visionDeviceManager->getDepthShortPixels());
		touchTracker->updateFrame(visionDeviceManager->getDepthShortPixels(), visionDeviceManager->getIrShortPixels(), visionDeviceManager->isDepthFrameNew());
	}
}

void TouchManager::setup()
{
	ofSetFrameRate(60);
	//setupWindow(); //호출하지 말 것!
	
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

void TouchManager::update(ofMatrix4x4 _system_pose)
{
	vector<FingerTouch> newTouches;
	if (touchTracker->update(newTouches)) {
		handleTouches(newTouches);
	}
	updateDebug();
	this->makeTouchPoint(_system_pose);

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
	if (!visionDeviceManager->isDepthFrameNew())
		return;

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

	//depthviz.draw(0, 0);	// depth 영상
	//drawText("Depth", 0, 0, HAlign::left, VAlign::top);

//	bgthread->drawDebug(0, dh);   //background 영상
	touchTracker->drawDebug(0, -dh);	// diff, diff+edge, edge, blob 영상
	
	// color영상 draw
//	this->colorTouchDraw(300, 300, 640, 360);
	/*
	drawText(ofVAArgsToString("FPS: %.1f\n", ofGetFrameRate())
		+ ofVAArgsToString("BG Update FPS: %.1f\n", bgthread->fps.fps)
		+ ofVAArgsToString("Touch Update FPS: %.1f\n", touchTracker->fps.fps), 
		ofGetWindowWidth(), 0, HAlign::right, VAlign::top);
		*/
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
	isTouchActivated = false;
}

void TouchManager::colorTouchDraw(int x, int y, int w, int h)
{
	ofImage colorImg = visionDeviceManager->getColorImage();
	float scale_x = w / colorImg.getWidth();
	float scale_y = h / colorImg.getHeight();
	
	vector<ofPoint> visited;
	vector<vector<ofPoint>> touch_buff;


	if (touchMap.size() > 0) {
		map<int, FingerTouch>::iterator iter;

		for (iter = touchMap.begin(); iter != touchMap.end(); iter++) {
			if (!iter->second.touched) continue;

			ofPoint pt = iter->second.tip;
			bool isNew = true;
			for (int i = 0; i < touch_buff.size(); ++i) {
				for (int j = 0; j < touch_buff.at(i).size(); ++j) {
					ofPoint pt1 = touch_buff.at(i).at(j);
					ofPoint distpt = pt1 - pt;
					float dist = sqrt(distpt.x*distpt.x + distpt.y*distpt.y);
					if (dist < 50) { //50 픽셀 이하면 같은 점으로 취급함
						isNew = false;
						touch_buff.at(i).push_back(pt1);
						break;
					}
					if (!isNew) break;
				}
			}
			if (isNew == true) {
				vector<ofPoint> temp;
				temp.push_back(pt);
				touch_buff.push_back(temp);
			}
		}

		for (int i = 0; i < touch_buff.size(); ++i) {
			if (touch_buff.at(i).size() == 1) {
				touchPoint.push_back(touch_buff.at(i).at(0));
			}
			else {
				ofPoint tot = ofPoint(0, 0, 0);
				for (int j = 0; j < touch_buff.at(i).size(); ++j) {
					tot += touch_buff.at(i).at(j);
				}
				tot = tot / touch_buff.at(i).size();
				touchPoint.push_back(tot);
			}
		}
		/*
		for (iter = touchMap.begin(); iter != touchMap.end(); iter++) {
			if (!iter->second.touched) continue;

			ofPoint pt = iter->second.tip;
			bool isNew = true;
			for (int i = 0; i < visited.size(); ++i) {
				ofPoint pt1 = visited.at(i);
				ofPoint distpt = pt1 - pt;
				float dist = sqrt(distpt.x*distpt.x + distpt.y*distpt.y);
				if (dist < 50) { //50 픽셀 이하면 같은 점으로 취급함
					isNew = false;
				}
			}

			if (isNew == true) {
				visited.push_back(pt);
				drawText("point" + ofToString(pt.x) + " " + ofToString(pt.y), pt*ofPoint(scale_x, scale_y) + ofPoint(x, y), HAlign::left, VAlign::top);
			}
		}
		*/
	}


	vector<ofPoint> touch_point_color =
		visionDeviceManager->depthToColor(touchPoint);

	ofPushStyle();
	ofSetColor(ofColor::green);
	for (ofPoint pt : touchPoint) {
		ofPoint draw_pt = 
			pt*ofPoint(scale_x, scale_y) + ofPoint(x, y);
		ofDrawCircle(draw_pt, 10);
	}
	ofPopStyle();

	colorImg.draw(x, y, w, h);
}

void TouchManager::meshDrawDebug()
{
	this->makeTouchMesh();
	ofPushMatrix();
	ofScale(-1, 1, -1);
	ofMultMatrix(system_pose);	
	visionDeviceManager->getPointCloudMesh().draw();
	ofPopMatrix();

	ofPushMatrix();
	glPointSize(10);
	touch_mesh->draw();
	glPointSize(1);
	ofPopMatrix();

}

bool TouchManager::getIsTouchActivate()
{
	return isTouchActivated;
}

void TouchManager::setIsTouchActivate(bool isTouchActivated)
{
	this->isTouchActivated = isTouchActivated;
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
				depth = 
					visionDeviceManager->getDepthShortPixels()[index];
					//depthStream.getShortPixelsRef().getPixels()[index]; // current (finger) depth
			}
			else {
				depth = bgthread->getBackgroundMean().getPixels()[index]; // stable (background) depth
			}

			float weight = (1 - fabsf(depthPos.x - x)) * (1 - fabsf(depthPos.y - y));

			/* Map depth point to camera point */
			ofVec3f wpt;
			HRESULT hr;

			//VisionDeviceManager 코드 변경 없이 아래 코드의 동작을 위한 구현이 필요하지만
			//getWorldPoint()는 어차피 프로젝션 출력 용이라고 해서 미구현 상태로 넘어감
			/*{
				DepthSpacePoint dpt_temp = { dpt.x, dpt.y };
				CameraSpacePoint wpt_temp;
				hr = device_->getMapper()->MapDepthPointToCameraSpace(dpt_temp, depth, &wpt_temp);
				*wpt = { wpt_temp.X, wpt_temp.Y, wpt_temp.Z };
				return SUCCEEDED(hr);
			}*/

			if (!SUCCEEDED(hr)) {
				ofLogError() << "MapDepthPointToCameraSpace failed";
				return ofPoint(0, 0, 0);
			}

			ret += weight * ofPoint(wpt.x, wpt.y, wpt.z);
		}
	}

	return ret;
}