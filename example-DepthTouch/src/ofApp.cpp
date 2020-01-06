#include "ofApp.h"

#define USE_KINECT2 true
#define USE_TOUCH true

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableAlphaBlending();
	ofEnableAntiAliasing();
	ofEnableDepthTest();

	/*
	setFlipHorizontal(false) -> 영상이 거울상처럼 보임 (카메라가 나를 바라봄)
	setFlipHorizontal(true) -> 카메라가 실제 보는 시야 (카메라와 내가 같은 방향을 바라봄)

	setFlip※(@) 함수는 받아오는 영상과 픽셀을 반전시킬 뿐 카메라 좌표계의 mesh들은 방향 불변임 (로컬 카메라 좌표계)
	*/

#if USE_KINECT2
	manager = VisionDeviceManager::Kinect2_Ptr();

	manager->setup();
	manager->setFlipHorizontal(true);
	manager->setFlipVertical(false);

	manager->setMapColorToDepth(true);
	manager->setMapDepthToCamera(true);
//	manager->setMapCameraToCloud(true);

#endif

#if USE_TOUCH
	touchManager.init(manager);
	touchManager.setup();
#endif

}

//--------------------------------------------------------------
void ofApp::update(){
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
#if USE_KINECT2
	manager->update();
#endif
#if USE_TOUCH
	if (touchManager.getIsTouchActivate()) {
		touchManager.update();
		vector<ofPoint> touchPt = touchManager.getTouchPoint();
		int h = manager->getDepthImage().getHeight();
		int w = manager->getDepthImage().getWidth();
		if(isMouseMove){
			if (touchPt.size() == 1) {
				mouseClickTime = 0;
				mouseCurPos = ofPoint((touchPt.at(0).x / w) * 1920, (touchPt.at(0).y / h) * 1080);
				SetCursorPos(mouseCurPos.x, mouseCurPos.y);
			}
			else if (touchPt.size() == 2) {
				cout << "나 클릭중" << endl;
				mouseClickTime += 1;
				if (mouseClickTime > 10) {
					cout << "클릭했는데?" << endl;
					POINT pt;
					pt.x = mouseCurPos.x;
					pt.y = mouseCurPos.y;
					HWND window = WindowFromPoint(pt);
					mouse_event(MOUSEEVENTF_LEFTDOWN, pt.x, pt.y, 0, 0);
					mouse_event(MOUSEEVENTF_LEFTUP, pt.x, pt.y, 0, 0);
					mouseClickTime = 0;
				}
			}
		}
	}
#endif
}

//--------------------------------------------------------------
void ofApp::draw(){
	//ofClear(ofColor::black);
	//ofBackground(ofColor::black);

#if USE_TOUCH
	/* Draw debug info */
	if (touchManager.getIsTouchActivate()) {
		ofPushMatrix();
		ofPushStyle();
		//ofTranslate(PROJW, 0); //setupWindow()를 호줄하지 않는 이상 주석처리 할 것!
		touchManager.drawDebug();
//		touchManager.meshDrawDebug();
		ofPopStyle();
		ofPopMatrix();
	}
	///*

	viewer.begin();
	ofSetBackgroundColor(ofColor::black);

	ofDrawAxis(2000);

	ofPushStyle();
	ofSetColor(ofColor::white);
	ofPushMatrix();
	ofRotate(90, 0, 0, -1);
	ofDrawGridPlane(500, 7, true);
	ofPopMatrix();
	ofPopStyle();
	touchManager.meshDrawDebug();
	viewer.end();
	//*/
#endif


//	ofPopMatrix();

//	viewer.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	constexpr float move_offset = 10.f;
	constexpr float angle_offset = 1.f;
	viewer.keyPressed(key);
	switch (key) {
	case OF_KEY_LEFT_SHIFT: shift_pressed = true; break;
	}

#if USE_TOUCH
	if (key == OF_KEY_ESC) {
		touchManager.stopThread();
		touchManager.teardown();
	}
	if (key == 't' || key == 'T') {
		touchManager.setIsTouchActivate(true);
		touchManager.startThread();
	}
	if (key == 'm' || key == 'M') {
		isMouseMove = !isMouseMove;
	}
#endif
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	switch (key) {
	case OF_KEY_LEFT_SHIFT:
		shift_pressed = false;
		break;
	}
}

void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	if (shift_pressed == false) {
		viewer.mouseDragged(x, y, button);
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	viewer.mousePressed(x, y, button);

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::moveMouse(ofPoint pt)
{
	SetCursorPos(0,0);
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
