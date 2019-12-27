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
	touchManager.startThread();
#endif

}

//--------------------------------------------------------------
void ofApp::update(){
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
#if USE_KINECT2
	manager->update();
#endif
#if USE_TOUCH
	touchManager.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofClear(ofColor::black);
	ofBackground(ofColor::black);

#if USE_TOUCH
	/* Draw debug info */
	ofPushMatrix();
	ofPushStyle();
	//ofTranslate(PROJW, 0); //setupWindow()를 호줄하지 않는 이상 주석처리 할 것!
	touchManager.drawDebug();
	ofPopStyle();
	ofPopMatrix();
#endif


//	ofPopMatrix();

//	viewer.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
#if USE_TOUCH
	if (key == OF_KEY_ESC) {
		touchManager.stopThread();
		touchManager.teardown();
	}
#endif
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

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
