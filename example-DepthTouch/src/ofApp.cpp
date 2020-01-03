#include "ofApp.h"

#define USE_KINECT2 true
#define USE_TOUCH true

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableAlphaBlending();
	ofEnableAntiAliasing();
	ofEnableDepthTest();

	/*
	setFlipHorizontal(false) -> ������ �ſ��ó�� ���� (ī�޶� ���� �ٶ�)
	setFlipHorizontal(true) -> ī�޶� ���� ���� �þ� (ī�޶�� ���� ���� ������ �ٶ�)

	setFlip��(@) �Լ��� �޾ƿ��� ����� �ȼ��� ������ų �� ī�޶� ��ǥ���� mesh���� ���� �Һ��� (���� ī�޶� ��ǥ��)
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
		//ofTranslate(PROJW, 0); //setupWindow()�� ȣ������ �ʴ� �̻� �ּ�ó�� �� ��!
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

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
