#pragma once
#include "TouchDeviceManager.h"
#include "ofxKinect2.h"

class TouchDeviceKinect2 : public TouchDeviceManager {
	const string TAG = typeid(*this).name();

protected:
	void configure() {
		DEPTH_WIDTH = depth_.getWidth();
		DEPTH_HEIGHT = depth_.getHeight();

		COLOR_WIDTH = color_.getWidth();
		COLOR_HEIGHT = color_.getHeight();

		calibrateCameras();

		DEPTH_HFOV = getDepthCalibrationResult().fovx;
		DEPTH_VFOV = getDepthCalibrationResult().fovy;

		PROJECTION_WIDTH = projectorCalib.resolutionX;
		PROJECTION_HEIGHT = projectorCalib.resolutionY;

		color2depth_aligned.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
		color2depth_buff.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);

		__super::allocateAfterConfigure();
	}

	ofxKinect2::Device* device_;
	ofxKinect2::IrStream ir_;
	ofxKinect2::ColorStream color_;
	ofxKinect2::DepthStream depth_;
	ofxKinect2::BodyStream body_stream_;

public:
	typedef shared_ptr<TouchDeviceKinect2> Ptr;
	inline string getDeviceType() {
		string type = string(TAG).replace(TAG.find("class "), string("class ").length(), "");
		return type;
	}
	inline ofxKinect2::Device& getDevice() { return *device_; }

	void exit() {
		cout << "ofxDepthDeviceKinect2 exitting..." << endl;
		waitForThread(true);
		color_.close();
		depth_.close();
		ir_.close();
		body_stream_.close();
		if(device_) device_->exit();
		cout << "ofxDepthDeviceKinect2 exit: success" << endl;
	}
	~TouchDeviceKinect2() {
		exit();
	}

	void setup() {
		ofLogNotice(TAG) << "setting up...";
		device_ = new ofxKinect2::Device();

		if (device_->setup()) {
			if (depth_.setup(*device_)) depth_.open();
			else ofLogError(TAG) << "DepthStream not opened";
			if (color_.setup(*device_)) color_.open();
			else ofLogError(TAG) << "ColorStream not opened";
			if (ir_.setup(*device_)) ir_.open();
			else ofLogError(TAG) << "IrStream not opened";
			if (body_stream_.setup(*device_)) body_stream_.open();
			else ofLogError(TAG) << "BodyStream not opened";

			while (!device_->isReady()) {
				update();
			}

			configure();
			startThread(false);
			waitForReady();
			ofLogNotice(TAG) << "connected";
		}
		else {
			ofLogFatalError(TAG) << "Device not found/connected!";
			FILE_LINE_FUNCTION;
			system("pause");
		}
	}

	inline bool isReady() { 
		return device_->isReady() && isThreadRunning() && bThreadRan; 
	}
	inline void waitForReady() {
		while (!isReady()) {
			this->update();
		}
	}
	void setTextureUsage(bool tf) {
		bUseTexture = tf;
		device_->update();
	}

	void update() {
		//stream들은 픽셀을 thread로 받아오고 update는 텍스처만 만듦
		if (bUseTexture) {
			depth_.update();
			color_.update();
			ir_.update();
			device_->update();
		}
	}

	void threadedFunction() {
		while (isThreadRunning()) {
			if (true) {
				ARLoopTimer vsync(TAG, false);
				if (bRefineDepthData) depthFrameRefine();					// hole filling 용도. filter는 엣지가 부드러워져서 지오메트리가 손상되므로 쓰지 않음
//				if (bMapColorToDepth) colorFrameToDepthSpace();				// 지오메트리에 색상 입히는 시각화 용도. 필요 없으면 끄는 게 퍼포먼스에 좋음
				if (bMapDepthToCamera) depthFrameToCameraSpace();			// 필수
//				if (bMapDepthToProjection) depthFrameToProjectionSpace();	// 지오메트리에 프로젝션 입히는 시각화 용도. 필요 없으면 끄는 게 퍼포먼스에 좋음
//				if (bMapCameraToCloud) cameraFrameToCloudMesh();
				
				bThreadRan = true;
				vsync.stall();
			}
		}
	}

	void draw() {
		getDepthImage().draw(0, 0);
		getColorImageAligned().draw(DEPTH_WIDTH, 0);
	}

	inline ofPixels& getColorPixels() { 
		return color_.getPixelsRef(); 
	} // 8비트  RGBA
	inline ofPixels& getColorPixelsUnflipped() { 
		return color_.getPixelsUnflipped(); 
	}
	inline ofPixels& getColorPixelsAligned() { 
		return color2depth_aligned; 
	}
	inline ofShortPixels& getDepthShortPixels() { 
		return depth_.getShortPixelsRef(); 
	} // 16비트 그레이
	inline ofShortPixels& getDepthShortPixelsUnflipped() { 
		return depth_.getShortPixelsUnflipped(); 
	}
	inline ofPixels getDepthPixels() { 
		return depth_.getPixels(); 
	} // 8비트 그레이

	inline void setFlipVertical(bool tf) {
		FLIP_VERTICAL = tf;
		color_.setMirror(FLIP_VERTICAL, FLIP_HORIZONTAL);
		depth_.setMirror(FLIP_VERTICAL, FLIP_HORIZONTAL);
		ir_.setMirror(FLIP_VERTICAL, FLIP_HORIZONTAL);
	}
	inline void setFlipHorizontal(bool tf) {
		FLIP_HORIZONTAL = tf;
		color_.setMirror(FLIP_VERTICAL, FLIP_HORIZONTAL);
		depth_.setMirror(FLIP_VERTICAL, FLIP_HORIZONTAL);
		ir_.setMirror(FLIP_VERTICAL, FLIP_HORIZONTAL);
	}

	inline void getFaceData() {
		// TBD
	}
	inline vector<ofxKinect2::Body> getSkeletons() {
		return body_stream_.getBodies();
	}

	inline int convertJointTableIndex(CommonJointTable common_idx) {
		int return_idx = -1;
		switch (common_idx) {
		case Head: return_idx = JointType_Head;
			break;
		case Neck: return_idx = JointType_Neck;
			break;
		case Body: return_idx = JointType_SpineMid;
			break;
		case L_Shoulder: return_idx = JointType_ShoulderLeft;
			break;
		case L_Elbow: return_idx = JointType_ElbowLeft;
			break;
		case L_Hand: return_idx = JointType_HandLeft;
			break;
		case R_Shoulder: return_idx = JointType_ShoulderRight;
			break;
		case R_Elbow: return_idx = JointType_ElbowRight;
			break;
		case R_Hand: return_idx = JointType_HandRight;
			break;
		case L_Hip: return_idx = JointType_HipLeft;
			break;
		case L_Knee: return_idx = JointType_KneeLeft;
			break;
		case L_Foot: return_idx = JointType_FootLeft;
			break;
		case R_Hip: return_idx = JointType_HipRight;
			break;
		case R_Knee: return_idx = JointType_KneeRight;
			break;
		case R_Foot: return_idx = JointType_FootRight;
			break;
		}
		return return_idx;
	}
	inline vector<Skeletons> getCommonSkeletons() {
		auto bodies = this->getSkeletons();
		vector<Skeletons> common_skeletons;
		for (int i = 0; i < bodies.size(); i++) {
			auto body = bodies[i];
			Skeletons common_skeleton;
			common_skeleton.player_index = i;

			for (int common_idx = 0; common_idx < CommonJointTable::COUNT; common_idx++) {
				Joint joint = body.getJoint(convertJointTableIndex((CommonJointTable)common_idx));
				
				Joints common_joint;
				common_joint.joint_id = common_idx;
				common_joint.point_3d = { joint.Position.X * 1000, joint.Position.Y * 1000, joint.Position.Z * 1000 };
				common_joint.point_2d = cameraToDepth(common_joint.point_3d);

				TrackingState prev_state = joint.TrackingState;
				CommonJointTrackingStatus common_state;
				switch (prev_state) {
				case TrackingState::TrackingState_NotTracked: common_state = NotTracked; break;
				case TrackingState::TrackingState_Tracked: common_state = Tracked; break;
				case TrackingState::TrackingState_Inferred: common_state = Inferred; break;
				}
				common_joint.tracking_status = common_state;

				common_skeleton.joints[common_idx] = common_joint;
			}
			common_skeletons.push_back(common_skeleton);
		}
		return common_skeletons;
	}

	inline ofPoint depthToCamera(const ofPoint& o) {
		return depthToCamera(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> depthToCamera(const vector<ofPoint>& o) {
		DepthSpacePoint* depths = new DepthSpacePoint[o.size()];
		UINT16* depthRaws = new UINT16[o.size()];
		for (size_t i = 0; i < o.size(); i++) {
			depths[i].X = reflipDepthX(o[i].x);
			depths[i].Y = reflipDepthY(o[i].y);
			depthRaws[i] = (bRefineDepthData ? depth_refined : getDepthShortPixelsUnflipped())[(int)(depths[i].Y + 0.5)*DEPTH_WIDTH + (int)(depths[i].X + 0.5)];
		}
		CameraSpacePoint* cameras = new CameraSpacePoint[o.size()];
		device_->getMapper()->MapDepthPointsToCameraSpace(o.size(), depths, o.size(), depthRaws, o.size(), cameras);

		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			out[i].x = cameras[i].X * 1000;
			out[i].y = cameras[i].Y * 1000;
			out[i].z = cameras[i].Z * 1000;
		}

		delete[] depths;
		delete[] depthRaws;
		delete[] cameras;
		return out;
	}

	inline ofPoint depthToColor(const ofPoint& o) {
		return depthToColor(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> depthToColor(const vector<ofPoint>& o) {
		DepthSpacePoint* depths = new DepthSpacePoint[o.size()];
		UINT16* depthRaws = new UINT16[o.size()];
		for (size_t i = 0; i < o.size(); i++) {
			depths[i].X = reflipDepthX(o[i].x);
			depths[i].Y = reflipDepthY(o[i].y);
			depthRaws[i] = (bRefineDepthData ? depth_refined : getDepthShortPixelsUnflipped())[(int)(depths[i].Y + 0.5)*DEPTH_WIDTH + (int)(depths[i].X + 0.5)];
		}
		ColorSpacePoint* colors = new ColorSpacePoint[o.size()];
		device_->getMapper()->MapDepthPointsToColorSpace(o.size(), depths, o.size(), depthRaws, o.size(), colors);

		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			out[i].x = FLIP_HORIZONTAL? COLOR_WIDTH - colors[i].X : colors[i].X;
			out[i].y = FLIP_VERTICAL? COLOR_HEIGHT - colors[i].Y : colors[i].Y;
		}

		delete[] depths;
		delete[] depthRaws;
		delete[] colors;
		return out;
	
	}
	
	inline ofPoint colorToCamera(const ofPoint& o) {
		return colorToCamera(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> colorToCamera(const vector<ofPoint>& o) {
		CameraSpacePoint* cameras = new CameraSpacePoint[COLOR_WIDTH*COLOR_HEIGHT];
		device_->getMapper()->MapColorFrameToCameraSpace(depth_.getWidth() * depth_.getHeight(), (bRefineDepthData ? depth_refined : getDepthShortPixelsUnflipped()).getData(), COLOR_WIDTH*COLOR_HEIGHT, cameras);
		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			ofPoint pt;
			pt.x = reflipColorX(o[i].x);
			pt.y = reflipColorY(o[i].y);

			CameraSpacePoint camera = cameras[(int)(pt.y+0.5)*COLOR_WIDTH + (int)(pt.x+0.5)];
			out[i].x = camera.X * 1000;
			out[i].y = camera.Y * 1000;
			out[i].z = camera.Z * 1000;
		}
		delete[] cameras;
		return out;
	}
	
	inline ofPoint colorToDepth(const ofPoint& o) {
		return colorToDepth(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> colorToDepth(const vector<ofPoint>& o) {
		DepthSpacePoint* depths = new DepthSpacePoint[COLOR_WIDTH * COLOR_HEIGHT];
		device_->getMapper()->MapColorFrameToDepthSpace(depth_.getWidth() * depth_.getHeight(), (bRefineDepthData ? depth_refined : getDepthShortPixelsUnflipped()).getData(), COLOR_WIDTH * COLOR_HEIGHT, depths);
		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			ofPoint pt;
			pt.x = reflipColorX(o[i].x);
			pt.y = reflipColorY(o[i].y);

			DepthSpacePoint depth = depths[(int)(pt.y + 0.5)*COLOR_WIDTH + (int)(pt.x + 0.5)];
			out[i].x = reflipDepthX(depth.X);
			out[i].y = reflipDepthY(depth.Y);
		}
		delete[] depths;
		return out;
	}
	
	inline ofPoint cameraToDepth(const ofPoint& o) {
		return cameraToDepth(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> cameraToDepth(const vector<ofPoint>& o) {
		CameraSpacePoint* cameras = new CameraSpacePoint[o.size()];
		for (size_t i = 0; i < o.size(); i++) {
			cameras[i].X = o[i].x / 1000.;
			cameras[i].Y = o[i].y / 1000.;
			cameras[i].Z = o[i].z / 1000.;
		}
		DepthSpacePoint* depths = new DepthSpacePoint[o.size()];
		device_->getMapper()->MapCameraPointsToDepthSpace(o.size(), cameras, o.size(), depths);

		vector<ofPoint> w(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			w[i].x = reflipDepthX(depths[i].X);
			w[i].y = reflipDepthY(depths[i].Y);
		}

		delete[] cameras;
		delete[] depths;
		return w;
	}
	
	inline ofPoint cameraToColor(const ofPoint& o) {
		return cameraToColor(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> cameraToColor(const vector<ofPoint>& o) {
		CameraSpacePoint* cameras = new CameraSpacePoint[o.size()];
		for (size_t i = 0; i < o.size(); i++) {
			cameras[i].X = o[i].x / 1000.;
			cameras[i].Y = o[i].y / 1000.;
			cameras[i].Z = o[i].z / 1000.;
		}
		ColorSpacePoint* colors = new ColorSpacePoint[o.size()];
		device_->getMapper()->MapCameraPointsToColorSpace(o.size(), cameras, o.size(), colors);

		vector<ofPoint> w(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			w[i].x = FLIP_HORIZONTAL? COLOR_WIDTH - colors[i].X: colors[i].X;
			w[i].y = FLIP_VERTICAL ? COLOR_HEIGHT - colors[i].Y : colors[i].Y;
		}
		
		delete[] cameras;
		delete[] colors;
		return w;
	}

	inline ofPoint depthToCamera(const ofPoint& o, const cv::Mat& depth16UC1) {
		return depthToCamera(vector<ofPoint>{ o }, depth16UC1)[0];
	}
	vector<ofPoint> depthToCamera(const vector<ofPoint>& o, const cv::Mat& depth16UC1) {
		DepthSpacePoint* depths = new DepthSpacePoint[o.size()];
		UINT16* depthRaws = new UINT16[o.size()];
		for (size_t i = 0; i < o.size(); i++) {
			depths[i].X = reflipDepthX(o[i].x);
			depths[i].Y = reflipDepthY(o[i].y);
			depthRaws[i] = ((ushort*)depth16UC1.data)[(int)(depths[i].Y + 0.5)*DEPTH_WIDTH + (int)(depths[i].X + 0.5)];
		}
		CameraSpacePoint* cameras = new CameraSpacePoint[o.size()];
		device_->getMapper()->MapDepthPointsToCameraSpace(o.size(), depths, o.size(), depthRaws, o.size(), cameras);

		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			out[i].x = cameras[i].X * 1000;
			out[i].y = cameras[i].Y * 1000;
			out[i].z = cameras[i].Z * 1000;
		}

		delete[] depths;
		delete[] depthRaws;
		delete[] cameras;
		return out;
	}

	inline ofPoint depthToColor(const ofPoint& o, const cv::Mat& depth16UC1) {
		return depthToColor(vector<ofPoint>{ o }, depth16UC1)[0];
	}
	vector<ofPoint> depthToColor(const vector<ofPoint>& o, const cv::Mat& depth16UC1) {
		DepthSpacePoint* depths = new DepthSpacePoint[o.size()];
		UINT16* depthRaws = new UINT16[o.size()];
		for (size_t i = 0; i < o.size(); i++) {
			depths[i].X = reflipDepthX(o[i].x);
			depths[i].Y = reflipDepthY(o[i].y);
			depthRaws[i] = ((ushort*)depth16UC1.data)[(int)(depths[i].Y + 0.5)*DEPTH_WIDTH + (int)(depths[i].X + 0.5)];
		}
		ColorSpacePoint* colors = new ColorSpacePoint[o.size()];
		device_->getMapper()->MapDepthPointsToColorSpace(o.size(), depths, o.size(), depthRaws, o.size(), colors);

		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			out[i].x = FLIP_HORIZONTAL ? COLOR_WIDTH - colors[i].X : colors[i].X;
			out[i].y = FLIP_VERTICAL ? COLOR_HEIGHT - colors[i].Y : colors[i].Y;
		}

		delete[] depths;
		delete[] depthRaws;
		delete[] colors;
		return out;
	}

	inline ofPoint colorToCamera(const ofPoint& o, const cv::Mat& depth16UC1) {
		return colorToCamera(vector<ofPoint>{ o }, depth16UC1)[0];
	}
	vector<ofPoint> colorToCamera(const vector<ofPoint>& o, const cv::Mat& depth16UC1) {
		CameraSpacePoint* cameras = new CameraSpacePoint[COLOR_WIDTH*COLOR_HEIGHT];
		device_->getMapper()->MapColorFrameToCameraSpace(depth_.getWidth() * depth_.getHeight(), ((ushort*)depth16UC1.data), COLOR_WIDTH*COLOR_HEIGHT, cameras);
		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			ofPoint pt;
			pt.x = reflipColorX(o[i].x);
			pt.y = reflipColorY(o[i].y);

			CameraSpacePoint camera = cameras[(int)(pt.y + 0.5)*COLOR_WIDTH + (int)(pt.x + 0.5)];
			out[i].x = camera.X * 1000;
			out[i].y = camera.Y * 1000;
			out[i].z = camera.Z * 1000;
		}
		delete[] cameras;
		return out;
	}

	inline ofPoint colorToDepth(const ofPoint& o, const cv::Mat& depth16UC1) {
		return colorToDepth(vector<ofPoint>{ o }, depth16UC1)[0];
	}
	vector<ofPoint> colorToDepth(const vector<ofPoint>& o, const cv::Mat& depth16UC1) {
		DepthSpacePoint* depths = new DepthSpacePoint[COLOR_WIDTH * COLOR_HEIGHT];
		device_->getMapper()->MapColorFrameToDepthSpace(depth_.getWidth() * depth_.getHeight(), ((ushort*)depth16UC1.data), COLOR_WIDTH * COLOR_HEIGHT, depths);
		vector<ofPoint> out(o.size());
		for (size_t i = 0; i < o.size(); i++) {
			ofPoint pt;
			pt.x = reflipColorX(o[i].x);
			pt.y = reflipColorY(o[i].y);

			DepthSpacePoint depth = depths[(int)(pt.y + 0.5)*COLOR_WIDTH + (int)(pt.x + 0.5)];
			out[i].x = reflipDepthX(depth.X);
			out[i].y = reflipDepthY(depth.Y);
		}
		delete[] depths;
		return out;
	}

protected:
	ofPixels color2depth_aligned;
	ofPixels color2depth_buff;
	void colorFrameToDepthSpace() {
		memset(color2depth_buff.getData(), 0, color2depth_buff.getTotalBytes());
		ofPixels& colors = getColorPixelsUnflipped();

		int depth_count = DEPTH_WIDTH * DEPTH_HEIGHT;
		vector<ColorSpacePoint> colorSpacePoints(depth_count);

		auto* mapper = device_->getMapper();
		mapper->MapDepthFrameToColorSpace(depth_count, getDepthShortPixelsUnflipped().getData(), depth_count, colorSpacePoints.data());

#ifdef CHECK_FOR_DUPLICATE
		vector<ofPoint> duplicate_checker(COLOR_HEIGHT*COLOR_WIDTH, ofPoint());
		for (int y = 0; y < DEPTH_HEIGHT; y++) {
			for (int x = 0; x < DEPTH_WIDTH; x++) {
				int depth_index = y * DEPTH_WIDTH + x;

				int depth_z = getDepthShortPixelsUnflipped()[depth_index];
				if (depth_z < MIN_DEPTH) continue;

				auto color_point = colorSpacePoints[depth_index];
				int color_index = (int)(color_point.Y + 0.5) * COLOR_WIDTH + (int)(color_point.X + 0.5);
				if (color_index < 0 || color_index >= COLOR_WIDTH*COLOR_HEIGHT) continue;

				ofPoint& dup = duplicate_checker[color_index];
				if (dup != ofPoint()) { //중복점 발견
					if (dup.z > depth_z) { //중복으로 체크된 false positive 매핑 발견
						int false_index = dup.y * DEPTH_WIDTH + dup.x;
						color2depth_buff[false_index * 3] = 0;
						color2depth_buff[false_index * 3 + 1] = 0;
						color2depth_buff[false_index * 3 + 2] = 0;
					}
					else { //자신이 중복되는 false negative 매핑임
						continue;
					}
				}

				color2depth_buff[depth_index * 3] = getColorPixelsUnflipped()[color_index * 4];
				color2depth_buff[depth_index * 3 + 1] = getColorPixelsUnflipped()[color_index * 4 + 1];
				color2depth_buff[depth_index * 3 + 2] = getColorPixelsUnflipped()[color_index * 4 + 2];
				duplicate_checker[color_index] = ofPoint(x, y, depth_z);

				if (color_index - COLOR_WIDTH - 1 >= 0)
					duplicate_checker[color_index - COLOR_WIDTH - 1] = ofPoint(x, y, depth_z);
				if (color_index - COLOR_WIDTH >= 0)
					duplicate_checker[color_index - COLOR_WIDTH] = ofPoint(x, y, depth_z);
				if (color_index - COLOR_WIDTH + 1 >= 0)
					duplicate_checker[color_index - COLOR_WIDTH + 1] = ofPoint(x, y, depth_z);

				if (color_index - 1 >= 0)
					duplicate_checker[color_index - 1] = ofPoint(x, y, depth_z);
				if (color_index + 1 < COLOR_WIDTH * COLOR_HEIGHT)
					duplicate_checker[color_index + 1] = ofPoint(x, y, depth_z);

				if (color_index + COLOR_WIDTH - 1 < COLOR_WIDTH * COLOR_HEIGHT)
					duplicate_checker[color_index + COLOR_WIDTH - 1] = ofPoint(x, y, depth_z);
				if (color_index + COLOR_WIDTH < COLOR_WIDTH * COLOR_HEIGHT)
					duplicate_checker[color_index + COLOR_WIDTH] = ofPoint(x, y, depth_z);
				if (color_index + COLOR_WIDTH + 1 < COLOR_WIDTH * COLOR_HEIGHT)
					duplicate_checker[color_index + COLOR_WIDTH + 1] = ofPoint(x, y, depth_z);
			}
		}
#else
		auto& depths = getDepthShortPixelsUnflipped();
		for (int y = 0; y < DEPTH_HEIGHT; y++) {
			for (int x = 0; x < DEPTH_WIDTH; x++) {
				int depth_index = y * DEPTH_WIDTH + x;
				if (!depths[depth_index]) continue;

				auto color_point = colorSpacePoints[depth_index]; //인덱스 자료형이 소수
				if (isnan(color_point.X) || isnan(color_point.Y) ||
					isinf(color_point.X) || isinf(color_point.Y)) continue; //inf, nan과 정수 연산하면 min_정수 나오므로 예외처리
				int color_index = ((int)(color_point.Y + 0.5) * COLOR_WIDTH + (int)(color_point.X + 0.5)) * colors.getNumChannels();
				if (color_index < 0 || color_index >= (COLOR_WIDTH * COLOR_HEIGHT) * colors.getNumChannels()) continue;

				int color2depth_index = reflipDepth(x, y) * color2depth_buff.getNumChannels();
				color2depth_buff[color2depth_index] = colors[color_index];
				color2depth_buff[color2depth_index + 1] = colors[color_index + 1];
				color2depth_buff[color2depth_index + 2] = colors[color_index + 2];
			}
		}
#endif
		memcpy(color2depth_aligned.getData(), color2depth_buff.getData(), color2depth_buff.getTotalBytes());
	}
};


TouchDeviceManager::Ptr TouchDeviceManager::Kinect2_Ptr() {
	return make_shared<TouchDeviceKinect2>();
}