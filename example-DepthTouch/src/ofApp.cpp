#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	this->setupKinect();

}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}


void ofApp::setupKinect()
{
	kinect = new ofxKinect2::Device();
	kinect->setup();
	kinect->setDepthColorSyncEnabled(); // needed to create the mapper object

	if (depthStream.setup(*kinect))
		depthStream.open();

#if 0
	if (colorStream.setup(*kinect))
		colorStream.open();
#endif
	if (irStream.setup(*kinect))
		irStream.open();

	/* Wait until the Kinect is running, then do the rest of the setup. */
	uint64_t start = ofGetElapsedTimeMillis();
	while (depthStream.getShortPixelsRef().getWidth() == 0 || depthStream.getShortPixelsRef().getHeight() == 0) {
		if (ofGetElapsedTimeMillis() - start > 5000) {
			throw std::runtime_error("Kinect failed to start in time!");
		}
		ofSleepMillis(20);
	}
}
