#pragma once

#include "ofMain.h"
#include "WindowUtils.h"
#include "geomConfig.h"
#include "Touch.h"
#include "ARUtils.hpp"

#ifndef FILE_LINE_FUNCTION
#define FILE_LINE_FUNCTION\
	std::cout << "\tFILE: " << __FILE__ << std::endl;\
	std::cout << "\tLINE: " << __LINE__ << std::endl;\
	std::cout << "\tFUNCTION: " << __FUNCTION__ << std::endl
#endif

class CalibrationResult {
	const string TAG = "CalibrationResult";
public:
	//http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
	//http://stackoverflow.com/questions/34023303/opencv-store-camera-matrix-and-distortion-coefficients-as-mat
	string	deviceType;
	int		resolutionX;
	int		resolutionY;
	float	reprojError;
	cv::Mat		cameraMatrix, distCoeffs;
	cv::Mat		rotationVec, rotationMat, translation;
	double	fovx, fovy;
	double	fx, fy, cx, cy;
	double	x, y, z;

	bool isLoaded() const {
		bool loaded = true;
		loaded &= !cameraMatrix.empty();
		loaded &= !distCoeffs.empty();
		loaded &= !rotationVec.empty();
		loaded &= !rotationMat.empty();
		loaded &= !translation.empty();
		return loaded;
	}

	//http://stackoverflow.com/questions/34023303/opencv-store-camera-matrix-and-distortion-coefficients-as-mat
	void setupCalibration() {
		if (isLoaded()) return;

		fx = cameraMatrix.at<double>(0, 0);
		fy = cameraMatrix.at<double>(1, 1);

		cx = cameraMatrix.at<double>(0, 2);
		cy = cameraMatrix.at<double>(1, 2);

		if (fx == 0 || fy == 0)  return;

		fovx = 2 * atan(resolutionX / (2 * fx)) * 180.0 / CV_PI;
		fovy = 2 * atan(resolutionY / (2 * fy)) * 180.0 / CV_PI;

		cv::Rodrigues(rotationVec, rotationMat);

		x = translation.at<double>(0);
		y = translation.at<double>(1);
		z = translation.at<double>(2);
	}
	bool loadCalibration(string path) {
		cv::FileStorage fs(ofToDataPath(path, true), cv::FileStorage::READ);
		if (!fs.isOpened()) return false;

		fs["deviceType"] >> deviceType;
		fs["intrisics"] >> cameraMatrix;
		fs["projResX"] >> resolutionX;
		fs["projResY"] >> resolutionY;
		fs["rotation"] >> rotationVec;
		fs["translation"] >> translation;
		fs["distCoeffs"] >> distCoeffs;
		fs["reprojectionError"] >> reprojError;
		setupCalibration();

		return true;
	}

	void projectPoints(IN const vector<cv::Point3f>& objectPoints, OUT vector<cv::Point2f>& imagePoints) const {
		if (objectPoints.empty()) return;
		cv::projectPoints(objectPoints, rotationVec, translation, cameraMatrix, distCoeffs, imagePoints);
	}

	void projectPoints(IN const vector<ofPoint>& objectPoints, OUT vector<cv::Point2f>& imagePoints) const {
		if (objectPoints.empty()) return;
		cv::projectPoints(XYZ(objectPoints), rotationVec, translation, cameraMatrix, distCoeffs, imagePoints);
	}

	void projectPoints(IN const vector<cv::Point3f>& objectPoints, OUT vector<ofPoint>& imagePoints) const {
		if (objectPoints.empty()) return;
		vector<cv::Point2f> _imagePoints;
		cv::projectPoints(objectPoints, rotationVec, translation, cameraMatrix, distCoeffs, _imagePoints);
		imagePoints = XY(_imagePoints);
	}

	void projectPoints(IN const vector<ofPoint>& objectPoints, OUT vector<ofPoint>& imagePoints) const {
		if (objectPoints.empty()) return;
		vector<cv::Point2f> _imagePoints;
		cv::projectPoints(XYZ(objectPoints), rotationVec, translation, cameraMatrix, distCoeffs, _imagePoints);
		imagePoints = XY(_imagePoints);
	}

	ofVec3f offCenterProjection() const {
		/*
		https://github.com/Microsoft/RoomAliveToolkit/blob/master/ProCamCalibration/ProCamEnsembleCalibration/GraphicsTransforms.cs
		목표는 프로젝터 화면의 중심을 XY [-1, 1]로 정규화된 그래픽스 카메라 화면의 중심과 맞추는 것이다.
		1. 화면 중심은 (w/2, h/2) 투영 중심은 (cx, cy)
		2. 화면 중심을 투영 중심으로 옮기는 변환 적용 (cx - w/2, cy - h/2)
		3. 투영 범위를 2로 정규화 시키는 변환 적용 (cx - w/2, cy - h/2) / (w/2, h/2)
		4. 식을 정리하면 (2*cx/w - 1, 2*cy/h - 1)
		5. horizontal flip(-1)이 왜 나오는 지는 모르겠지만 넣긴 해야 함
		*/

		int w = resolutionX;
		int h = resolutionY;
		return ofPoint(-1, 1) * ofPoint(2 * cx / w - 1, 2 * cy / h - 1);
	}
	ofPoint originOfProjection() const {
		return reverseProjection(ofPoint(0, 0, 0));
	}
	ofPoint directionOfProjection() const {
		return reverseProjection(ofPoint(cx, cy, 1)) - originOfProjection();
	}

	ofPoint reverseIntrinsic(const ofPoint & o) const {
		cv::Mat pt = cv::Mat::zeros(3, 1, CV_64F);
		((double*)pt.data)[0] = o.x;
		((double*)pt.data)[1] = o.y;
		((double*)pt.data)[2] = 1;

		double s = o.z;
		const cv::Mat A = cameraMatrix;

		cv::Mat rp = A.inv() * s * pt;
		return ofPoint(((double*)rp.data)[0], ((double*)rp.data)[1], ((double*)rp.data)[2]);
	}
	ofPoint reverseExtrinsic(const ofPoint &o) const {
		cv::Mat pt = cv::Mat::zeros(3, 1, CV_64F);
		((double*)pt.data)[0] = o.x;
		((double*)pt.data)[1] = o.y;
		((double*)pt.data)[2] = o.z;

		cv::Mat rt = rotationMat.inv() * (pt - translation);
		return ofPoint(((double*)rt.data)[0], ((double*)rt.data)[1], ((double*)rt.data)[2]);
	}
	ofPoint reverseProjection(const ofPoint & o) const {
		return reverseExtrinsic(reverseIntrinsic(o));
	}

	ofCamera buildProjectorView() const {
		ofCamera projectorView;
		projectorView.setFov(fovy);
		projectorView.setFarClip(10000);
		projectorView.setLensOffset(offCenterProjection());

		/*
		fovx 36.6339 tan(fovx) 0.74358394678 tan(half_fovx) 0.33104660216
		fovy 21.0969 tan(fovy) 0.38580572604 tan(half_fovy) 0.18621384151
		tan(fovx)/tan(fovy) = 1.9273533195380979576712557182035
		tan(half_fovx)/tan(half_fovy) = 1.7777765577228706407561614778912
		16/9 = 1.7777777777777777777777777777778
		*/
		//해상도 비율 == tan(half_fov) 비율
		float aspect = tan(ofDegToRad(fovx / 2)) / tan(ofDegToRad(fovy / 2));
		projectorView.setForceAspectRatio(true);
		projectorView.setAspectRatio(aspect);

		//아래 if-else문은 같은 변환 결과를 낸다.
		if (true) {
			//rotationMat, translation: 키넥트(MS 좌표계)에서 프로젝터(CV 좌표계)로
			cv::Matx44d kinect2proj = {
				rotationMat.at<double>(0, 0), rotationMat.at<double>(0, 1), rotationMat.at<double>(0, 2), translation.at<double>(0),
				rotationMat.at<double>(1, 0), rotationMat.at<double>(1, 1), rotationMat.at<double>(1, 2), translation.at<double>(1),
				rotationMat.at<double>(2, 0), rotationMat.at<double>(2, 1), rotationMat.at<double>(2, 2), translation.at<double>(2),
				0, 0, 0, 1
			};
			cv::Mat proj2kinect = cv::Mat(kinect2proj).inv(); //프로젝터(CV 좌표계)에서 키넥트(MS 좌표계)로

			cv::Mat msgl = cv::Mat(cv::Matx44d(
				-1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, -1, 0,
				0, 0, 0, 1
			));
			cv::Mat cvgl = cv::Mat(cv::Matx44d(
				1, 0, 0, 0,
				0, -1, 0, 0,
				0, 0, -1, 0,
				0, 0, 0, 1
			));

			cv::Mat projPose = msgl * proj2kinect * cvgl;
			projPose = projPose.t();
			//cout << "projPose:" << endl << projPose << endl;

			projPose.convertTo(projPose, CV_32F);
			projectorView.setTransformMatrix(ofMatrix4x4((float*)projPose.data));
		}
		else {
			cv::Matx33d projR = cv::Mat(rotationMat.t());
			cv::Vec3d projt = cv::Mat(rotationMat.t() * -translation);
			cv::Matx44d projRt = {
				projR(0, 0), projR(0, 1), projR(0, 2), projt(0),
				projR(1, 0), projR(1, 1), projR(1, 2), projt(1),
				projR(2, 0), projR(2, 1), projR(2, 2), projt(2),
				0, 0, 0, 1
			}; //위와 아래는 본질적으로 같음

			   //(키넥트 좌표계 변환 * 비전 좌표계 좌표)*그래픽스 좌표계 변환
			cv::Vec4d origin = (projRt * cv::Vec4d(0, 0, 0, 1)).mul(cv::Vec4d(-1, 1, -1, 1));	//프로젝터 원점
			cv::Vec4d lookVec = (projRt * cv::Vec4d(0, 0, 1, 1)).mul(cv::Vec4d(-1, 1, -1, 1));	//프로젝터 앞
			cv::Vec4d upVec = (projRt * cv::Vec4d(0, -1, 0, 1)).mul(cv::Vec4d(-1, 1, -1, 1));	//프로젝터 위

			projectorView.move(ofVec3f(origin(0), origin(1), origin(2)));
			projectorView.lookAt(ofVec3f(lookVec(0), lookVec(1), lookVec(2)), ofVec3f(upVec(0) - origin(0), upVec(1) - origin(1), upVec(2) - origin(2)));
			//cout << "projectorView:" << endl << projectorView.getGlobalTransformMatrix() << endl;
		}

		return projectorView;
	}
};

/*
JointTable	KinectV2				Kinect1									OpenNi					Astra
Head		JointType_Head			NUI_SKELETON_POSITION_HEAD				JOINT_HEAD				ASTRA_JOINT_HEAD
Neck		JointType_Neck			NUI_SKELETON_POSITION_SHOULDER_CENTER	JOINT_NECK				ASTRA_JOINT_NECK
Body		JointType_SpineMid		NUI_SKELETON_POSITION_SPINE				JOINT_TORSO				ASTRA_JOINT_MID_SPINE

L_Shoulder	JointType_ShoulderLeft	NUI_SKELETON_POSITION_SHOULDER_LEFT		JOINT_LEFT_SHOULDER		ASTRA_JOINT_LEFT_SHOULDER
L_Elbow		JointType_ElbowLeft		NUI_SKELETON_POSITION_ELBOW_LEFT		JOINT_LEFT_ELBOW		ASTRA_JOINT_LEFT_ELBOW
L_Hand		JointType_HandLeft		NUI_SKELETON_POSITION_HAND_LEFT			JOINT_LEFT_HAND			ASTRA_JOINT_LEFT_HAND

R_Shoulder	JointType_ShoulderRight	NUI_SKELETON_POSITION_SHOULDER_RIGHT	JOINT_RIGHT_SHOULDER	ASTRA_JOINT_RIGHT_SHOULDER
R_Elbow		JointType_ElbowRight	NUI_SKELETON_POSITION_ELBOW_RIGHT		JOINT_RIGHT_ELBOW		ASTRA_JOINT_RIGHT_ELBOW
R_Hand		JointType_HandRight		NUI_SKELETON_POSITION_HAND_RIGHT		JOINT_RIGHT_HAND		ASTRA_JOINT_RIGHT_HAND

L_Hip		JointType_HipLeft		NUI_SKELETON_POSITION_HIP_LEFT			JOINT_LEFT_HIP			ASTRA_JOINT_LEFT_HIP
L_Knee		JointType_KneeLeft		NUI_SKELETON_POSITION_KNEE_LEFT			JOINT_LEFT_KNEE			ASTRA_JOINT_LEFT_KNEE
L_Foot		JointType_FootLeft		NUI_SKELETON_POSITION_FOOT_LEFT			JOINT_LEFT_FOOT			ASTRA_JOINT_LEFT_FOOT

R_Hip		JointType_HipRight		NUI_SKELETON_POSITION_HIP_RIGHT			JOINT_RIGHT_HIP			ASTRA_JOINT_RIGHT_HIP
R_Knee		JointType_KneeRight		NUI_SKELETON_POSITION_KNEE_RIGHT		JOINT_RIGHT_KNEE		ASTRA_JOINT_RIGHT_KNEE
R_Foot		JointType_FootRight		NUI_SKELETON_POSITION_FOOT_RIGHT		JOINT_RIGHT_FOOT		ASTRA_JOINT_RIGHT_FOOT
*/
enum CommonJointTable {
	Head,
	Neck,
	Body,
	L_Shoulder,
	L_Elbow,
	L_Hand,
	R_Shoulder,
	R_Elbow,
	R_Hand,
	L_Hip,
	L_Knee,
	L_Foot,
	R_Hip,
	R_Knee,
	R_Foot,
	COUNT
};
enum CommonJointTrackingStatus {
	NotTracked,
	Inferred,
	Tracked
};
struct Joints {
	ofPoint point_3d;
	ofPoint point_2d;

	int joint_id = -1;
	float confidence = 0;
	CommonJointTrackingStatus tracking_status = Tracked;

	bool isTracked() { return tracking_status != CommonJointTrackingStatus::NotTracked; }

	void drawJoint3D();
	void drawJoint2D();
};
struct Skeletons {
	vector<Joints> joints = vector<Joints>(CommonJointTable::COUNT);
	int player_index = -1;

	Joints getJoint(CommonJointTable query) { return joints[query]; }
	void drawJoints3D();
	void drawJoints2D();
};

class TouchResult {
	const string TAG = "TouchResult";

public:
	string deviceType;

	ofPoint getWorldPoint(const ofVec2f &depthPt, bool live);
	ofPoint getBackgroundWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, false); }
	ofPoint getLiveWorldPoint(const ofVec2f &depthPt) { return getWorldPoint(depthPt, true); }

	class TouchTracker *touchTracker;
	map <int, FingerTouch> touchMap;
	
	void drawProjector() {
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
	//draw debug
	void drawDebug() {
		/*
		const int dw = depthStream.getWidth();
		const int dh = depthStream.getHeight();

		depthviz.draw(0, 0);
		drawText("Depth", 0, 0, HAlign::left, VAlign::top);

		bgthread->drawDebug(0, dh);
		touchTracker->drawDebug(dw, 0);

		drawText(ofVAArgsToString("FPS: %.1f\n", ofGetFrameRate())
			+ ofVAArgsToString("BG Update FPS: %.1f\n", bgthread->fps.fps)
			+ ofVAArgsToString("Touch Update FPS: %.1f\n", touchTracker->fps.fps), DISPW, 0, HAlign::right, VAlign::top);

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

	void setupDebug();
	void handleTouches(const vector<FingerTouch> &newTouches) {
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
	void updateDebug();

	/* Debugging */
	ofImage depthviz;
	uint64_t lastDepthTimestamp;
	int curDepthFrame;

};

class TouchDeviceManager : public ofThread {
public:
	typedef shared_ptr<TouchDeviceManager> Ptr;

	static inline Ptr Kinect2_Ptr();

private:
	const string TAG = typeid(*this).name();
protected: // 중요 공통 변수

	int DEPTH_WIDTH, DEPTH_HEIGHT;
	int COLOR_WIDTH, COLOR_HEIGHT;
	int PROJECTION_WIDTH, PROJECTION_HEIGHT;

	double DEPTH_HFOV, DEPTH_VFOV;

	bool IS_DEPTH_NEW;

	//Lachat, Elise, et al. "Assessment and calibration of a RGB-D camera (Kinect v2 Sensor) towards a potential use for close-range 3D modeling." Remote Sensing 7.10 (2015): 13070-13097.
	//Operative measuring range from 0.5 to 4.5 m
	//Object pixel size(GSD) between 1.4 mm(@ 0.5 m range) and 12 mm(@ 4.5 m range)
	int MIN_DEPTH = 500;
	int MAX_DEPTH = 4500;

	bool FLIP_VERTICAL = false;
	bool FLIP_HORIZONTAL = false;

	virtual void configure() = 0;

	//--캘리브레이션---
	CalibrationResult projectorCalib;
	CalibrationResult depthCalib;
	CalibrationResult colorCalib;

	//이미지
	ofImage colorImage;
	ofImage colorImageAligned;
	ofImage depthImage;
	ofShortImage shortImage;

	TouchResult touchResult;
	
public: // 중요 공통 함수
	inline float reflipDepthX(float x) {
		return FLIP_HORIZONTAL ? DEPTH_WIDTH - x - 1 : x;
	}
	inline float reflipDepthY(float y) {
		return FLIP_VERTICAL ? DEPTH_HEIGHT - y - 1 : y;
	}
	inline int reflipDepth(float x, float y) {
		return DEPTH_WIDTH * (int)(reflipDepthY(y) + 0.5) + (int)(reflipDepthX(x) + 0.5);
	}
	inline float reflipColorX(float x) {
		return FLIP_HORIZONTAL ? COLOR_WIDTH - x - 1 : x;
	}
	inline float reflipColorY(float y) {
		return FLIP_VERTICAL ? COLOR_HEIGHT - y - 1 : y;
	}
	inline int reflipColor(float x, float y) {
		return COLOR_WIDTH * (int)(reflipColorY(y) + 0.5) + (int)(reflipColorX(x) + 0.5);
	}

protected:
	void allocateAfterConfigure();

	void calibrateCameras();
	void calibrateDepthCamera();
	void calibrateColorCamera();

public:
	virtual inline string getDeviceType() {
		ofLogError(TAG) << "Direct access to virtual base class should not happen";
		string type = string(TAG).replace(TAG.find("class "), string("class ").length(), "");
		return type;
	}

	inline const CalibrationResult& getColorCalibrationResult() { return colorCalib; }
	inline const CalibrationResult& getDepthCalibrationResult() { return depthCalib; }
	inline const CalibrationResult& getProjectorCalibrationResult() { return projectorCalib; }


	virtual inline ofPixels& getColorPixels() = 0; // 8비트 RGBA
	virtual inline ofPixels& getColorPixelsUnflipped() = 0;
	virtual inline ofPixels& getColorPixelsAligned() = 0; // 8비트 RGBA
	virtual inline ofImage& getColorImage() {
		colorImage.setFromPixels(getColorPixels());
		return colorImage;
	}
	virtual inline ofImage& getColorImageAligned() {
		colorImageAligned.setFromPixels(getColorPixelsAligned());
		return colorImageAligned;
	}
	virtual inline cv::Mat getColorMat() {
		auto& pixels = getColorPixels();
		int channels = pixels.getNumChannels();
		ofPixelFormat format = pixels.getPixelFormat();
		int from_type;
		int to_type;
		std::tie(from_type, to_type) = ARUtils::ofPixelType2cvMatType(format);
		cv::Mat color = cv::Mat(pixels.getHeight(), pixels.getWidth(), from_type, pixels.getData()).clone();
		if (to_type != -1) cvtColor(color, color, to_type);
		return color;
	}
	virtual inline cv::Mat getColorMatAligned() {
		auto& pixels = getColorPixelsAligned();
		int channels = pixels.getNumChannels();
		ofPixelFormat format = pixels.getPixelFormat();
		int from_type;
		int to_type;
		std::tie(from_type, to_type) = ARUtils::ofPixelType2cvMatType(format);
		cv::Mat color = cv::Mat(pixels.getHeight(), pixels.getWidth(), from_type, pixels.getData()).clone();
		if (to_type != -1) cvtColor(color, color, to_type);
		return color;
	}

	virtual inline ofPixels getDepthPixels() = 0; // 8비트 그레이
	virtual inline ofPixels getDepthPixelsRefined() {
		return depth_refined_8bit;
	}
	virtual inline ofShortPixels& getDepthShortPixels() = 0; // 16비트 그레이
	virtual inline ofShortPixels& getDepthShortPixelsUnflipped() = 0;
	virtual inline ofShortPixels& getDepthShortPixelsRefined() {
		return bRefineDepthData ? depth_refined : getDepthShortPixelsUnflipped();
	}
	virtual inline ofImage& getDepthImage() {
		depthImage.setFromPixels(getDepthPixels());
		return depthImage;
	}
	virtual inline cv::Mat getDepthMat() {
		auto& pixels = getDepthPixels();
		cv::Mat depth8 = cv::Mat(pixels.getHeight(), pixels.getWidth(), CV_8UC1, pixels.getData()).clone();
		return depth8;
	}

	inline ofShortImage& getDepthShortImage() {
		shortImage.setFromPixels(getDepthShortPixels());
		return shortImage;
	}
	inline cv::Mat getDepthShortMat() {
		auto& pixels = getDepthShortPixels();
		cv::Mat depth16 = cv::Mat(pixels.getHeight(), pixels.getWidth(), CV_16UC1, pixels.getData()).clone();
		return depth16;
	}

	inline const ofPoint* getDepthToCameraTable() { return depth2camera_table.data(); }
	inline const ofPoint* getDepthToProjectionTable() { return depth2projection_table.data(); }

	inline const vector<ofPoint>& getDepthToCameraMapping() { return depth2camera_table; }
	inline const vector<ofPoint>& getDepthToProjectionMapping() { return depth2projection_table; }

	virtual int convertJointTableIndex(CommonJointTable common_idx) = 0;
	virtual vector<Skeletons> getCommonSkeletons() = 0;

	inline int getDepthWidth() { return DEPTH_WIDTH; }
	inline int getDepthHeight() { return DEPTH_HEIGHT; }

	inline double getDepthHFOV() { return DEPTH_HFOV; }
	inline double getDepthVFOV() { return DEPTH_VFOV; }

	inline int getMinDepth() { return MIN_DEPTH; }
	inline int getMaxDepth() { return MAX_DEPTH; }

	inline int getColorWidth() { return COLOR_WIDTH; }
	inline int getColorHeight() { return COLOR_HEIGHT; }

	inline int getProjectionWidth() { return PROJECTION_WIDTH; }
	inline int getProjectionHeight() { return PROJECTION_HEIGHT; }

	inline bool isFlipVertical() { return FLIP_VERTICAL; }
	inline bool isFlipHoritonzal() { return FLIP_HORIZONTAL; }

	virtual inline void setFlipVertical(bool tf) = 0;
	virtual inline void setFlipHorizontal(bool tf) = 0;

	inline bool isTextureUsed() { return bUseTexture; }
	virtual inline void setTextureUsage(bool tf) = 0;


	virtual void setup() = 0;
	virtual void update() = 0;
	virtual void threadedFunction() = 0;
	virtual void draw() = 0;
	virtual void exit() = 0;
	virtual bool isReady() = 0;

	virtual void drawFace() {
		ofLogWarning(TAG) << "drawFace(): do your own implementation in the derived class";
	}
	virtual void drawSkeletons() {
		ofLogWarning(TAG) << "drawSkeletons(): do your own implementation in the derived class";
	}
	virtual void drawPlayers() {
		ofLogWarning(TAG) << "drawPlayers(): do your own implementation in the derived class";
	}

protected: // boolean
	bool bThreadRan = false;
	bool bRefineDepthData = false;
	bool bFilterDepthData = false;
	bool bMapColorToDepth = true;
	bool bMapDepthToCamera = true;
	bool bMapDepthToProjection = false;
	bool bUseTexture = false;

public:
	inline void setRefineDepthData(bool tf) { bRefineDepthData = tf; }
	inline void setFilterDepthData(bool tf) { bFilterDepthData = tf; }
	inline void setMapColorToDepth(bool tf) { bMapColorToDepth = tf; }
	inline void setMapDepthToCamera(bool tf) { bMapDepthToCamera = tf; }
	inline void setMapDepthToProjection(bool tf) { bMapDepthToProjection = tf; }

	inline bool getRefineDepthData() { return bRefineDepthData; }
	inline bool getFilterDepthData() { return bFilterDepthData; }
	inline bool getMapColorToDepth() { return bMapColorToDepth; }
	inline bool getMapDepthToCamera() { return bMapDepthToCamera; }
	inline bool getMapDepthToProjection() { return bMapDepthToProjection; }

	inline void loadProjectorCalibration(string path) {
		projectorCalib.loadCalibration(path);
	}

	//http://stackoverflow.com/questions/3712049/how-to-use-an-opencv-rotation-and-translation-vector-with-opengl-es-in-android 
	//http://stackoverflow.com/questions/22938455/the-coordinate-system-of-pinhole-camera-model
	//http://stackoverflow.com/questions/17987465/how-is-the-camera-coordinate-system-in-opencv-oriented
	//오픈지엘과 오픈씨비는 변환 방식이 다르지만, 여기선 캘리브레이션 결과도 씨비고 변환도 씨비기 때문에 별 상관 없을 듯


public:

	inline ofPoint cameraToProjection(const ofPoint& o) {
		return cameraToProjection(vector<ofPoint>{ o })[0];
	}
	vector<ofPoint> cameraToProjection(const vector<ofPoint>& o);

	inline ofPoint depthToProjection(const ofPoint& o) {
		return depthToProjection(vector<ofPoint>{ o })[0];
	}
	inline vector<ofPoint> depthToProjection(const vector<ofPoint>& o) {
		return cameraToProjection(depthToCamera(o));
	}

	inline ofPoint colorToProjection(const ofPoint& o) {
		return colorToProjection(vector<ofPoint>{ o })[0];
	}
	inline vector<ofPoint> colorToProjection(const vector<ofPoint>& o) {
		return cameraToProjection(colorToCamera(o));
	}

	virtual ofPoint depthToCamera(const ofPoint& o) = 0;
	virtual vector<ofPoint> depthToCamera(const vector<ofPoint>& o) = 0;

	virtual ofPoint depthToColor(const ofPoint& o) = 0;
	virtual vector<ofPoint> depthToColor(const vector<ofPoint>& o) = 0;

	virtual ofPoint colorToCamera(const ofPoint& o) = 0;
	virtual vector<ofPoint> colorToCamera(const vector<ofPoint>& o) = 0;

	virtual ofPoint colorToDepth(const ofPoint& o) = 0;
	virtual vector<ofPoint> colorToDepth(const vector<ofPoint>& o) = 0;

	virtual ofPoint cameraToDepth(const ofPoint& o) = 0;
	virtual vector<ofPoint> cameraToDepth(const vector<ofPoint>& o) = 0;

	virtual ofPoint cameraToColor(const ofPoint& o) = 0;
	virtual vector<ofPoint> cameraToColor(const vector<ofPoint>& o) = 0;

	virtual ofPoint depthToCamera(const ofPoint& o, const cv::Mat& depth16UC1) = 0;
	virtual vector<ofPoint> depthToCamera(const vector<ofPoint>& o, const cv::Mat& depth16UC1) = 0;

	virtual ofPoint depthToColor(const ofPoint& o, const cv::Mat& depth16UC1) = 0;
	virtual vector<ofPoint> depthToColor(const vector<ofPoint>& o, const cv::Mat& depth16UC1) = 0;

	virtual ofPoint colorToCamera(const ofPoint& o, const cv::Mat& depth16UC1) = 0;
	virtual vector<ofPoint> colorToCamera(const vector<ofPoint>& o, const cv::Mat& depth16UC1) = 0;

	virtual ofPoint colorToDepth(const ofPoint& o, const cv::Mat& depth16UC1) = 0;
	virtual vector<ofPoint> colorToDepth(const vector<ofPoint>& o, const cv::Mat& depth16UC1) = 0;


protected :
	vector<ofPoint> depth2camera_table;
	vector<ofPoint> all_depth_points;	//부모 클래스에서 (로컬이라도) static이 있으면 모든 자식이 공유하므로 버퍼 선언할 때 주의
	vector<ofPoint> depth2projection_table;


	ofPixels depth_refined_8bit;
	ofShortPixels depth_refined;

	inline void depthFrameToCameraSpace() {
		memcpy(depth2camera_table.data(), depthToCamera(all_depth_points).data(), sizeof(ofPoint) * DEPTH_WIDTH * DEPTH_HEIGHT);
	}

	//http://fukushima.web.nitech.ac.jp/research/wjbf/
	void depthFrameRefine();
};

