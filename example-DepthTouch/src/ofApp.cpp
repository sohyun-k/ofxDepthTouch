#include "ofApp.h"

#define USE_KINECT2 true
#define USE_TOUCH true

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableDepthTest();
	viewer.setNearClip(0.1);
	viewer.setFarClip(20000);

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
//	ofClear(ofColor::black);
//	ofBackground(ofColor::black);

//	viewer.begin();
	/*
	ofDrawAxis(5000);
	ofPushMatrix();
	ofRotate(90, 0, 0, -1);
	ofPushStyle();
	ofSetColor(ofColor::white);
	ofDrawGridPlane(500, 10, true);
	ofPopStyle();
	ofPopMatrix();

	ofPushMatrix();
	*/

	/*
#if USE_KINECT2
	//skeletons...
	manager->draw();
	manager->getColorImage().draw(manager->getDepthWidth() * 2, 0, 640.f, manager->getColorHeight() / (manager->getColorWidth() / 640));
	ofPushMatrix();
	ofScale(1, -1, 1);
	ofTranslate(0, -manager->getDepthHeight());
	for (auto skeleton : manager->getCommonSkeletons()) skeleton.drawJoints2D();
	ofPopMatrix();
#endif
	ofPopMatrix();

	ofPushMatrix();
*/

#if USE_TOUCH
	/* Draw onto projector */
	/*
	ofPushMatrix();
	ofPushStyle();
	ofSetMatrixMode(OF_MATRIX_MODELVIEW);
	ofMultMatrix(projector_transpose);
	touchManager.drawProjector();
	ofPopStyle();
	ofPopMatrix();
	*/
	/* Draw debug info */
	ofPushMatrix();
	ofPushStyle();
	ofTranslate(PROJW, 0);
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
	int x_offset = x - previous_x;
	int y_offset = y - previous_y;

	if (button == 0) {
		// �¿� ȸ��(Pan) Y�࿡ ���� / ���� ȸ��(Tilt) X �࿡ ����
		float x_scale = (0.75f * 360) / ofGetWidth();
		float y_scale = (0.75f * 360) / ofGetHeight();

		float panVal = x_offset * x_scale;
		float tiltVal = -y_offset * y_scale;

		viewer.rotate(panVal, ofVec3f(0, -1 * abs(viewer.getPosition().y), 0));
		viewer.rotate(tiltVal, viewer.getXAxis()); //pitch-
	}
	else if (button == 2) {
		//x_offset�� roll�� ����
		float x_scale = ((float)1 * 360) / ofGetWidth();
		float y_scale = ((float)1000) / ofGetHeight();

		float elevationVal = -y_offset * y_scale;
		//boom(elevationVal);
		viewer.move(0, elevationVal, 0);
	}

	previous_x = x;
	previous_y = y;
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	static auto last = ofGetElapsedTimeMillis();
	auto now = ofGetElapsedTimeMillis();
	if (button == 0) {
		if (now - last < 500) {
			viewer.rotate(-viewer.getOrientationEuler().x, viewer.getXAxis());
		}
		last = now;
	}

	previous_x = x;
	previous_y = y;
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
