#pragma once

#include "ofMain.h"
#include "VisionDeviceManager.hpp"
#include "VisionDeviceKinect2.hpp"
#include "TouchManager.h"
#include <Windows.h>

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		VisionDeviceManager::Ptr manager;
		TouchManager touchManager;

		ARViewer viewer;
		bool shift_pressed;

		void moveMouse(ofPoint pt);
		bool isMouseMove = false;
		int mouseClickTime = 0;
		ofPoint mouseCurPos=ofPoint();
};
