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
	//ofTranslate(PROJW, 0); //setupWindow()�� ȣ������ �ʴ� �̻� �ּ�ó�� �� ��!
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
