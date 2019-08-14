#pragma once

#include "ofMain.h"
#include "ofxKinect2.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		
		ofxKinect2::Device* kinect;
		ofxKinect2::IrStream irStream;
		ofxKinect2::ColorStream colorStream;
		ofxKinect2::DepthStream depthStream;

		void setupKinect();



};
