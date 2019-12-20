#include "TouchDeviceManager.h"
#include "BilateralFilter/filter.h"
#include "BilateralFilter/util.h"
#include "BilateralFilter/viewSynthesis.h"

using namespace std;
using namespace cv;

void TouchDeviceManager::depthFrameRefine() {
	Mat srcDepth(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1, getDepthShortPixelsUnflipped().getData());
	Mat refinedDepth = srcDepth.clone();

	if (bRefineDepthData)
		fillOcclusionDepth(refinedDepth, 0);
	if (bFilterDepthData) {
		Mat filledDepthf; refinedDepth.convertTo(filledDepthf, CV_32F);
		Mat filteredDepthf = Mat::zeros(srcDepth.size(), CV_32F);

		int radius = 5; // flying pixel 최대한 제거
		float epsilon = 0.1; // 엣지 최대한 유지시키면서 뎁스 떨림 제거
		guidedFilterTBB(filledDepthf, filteredDepthf, radius, epsilon, 8);
		filteredDepthf.convertTo(refinedDepth, CV_16U);
	}

	memcpy(depth_refined.getData(), refinedDepth.data, sizeof(ushort)*DEPTH_WIDTH*DEPTH_HEIGHT);
	memcpy(depth_refined_8bit.getData(), ARUtils::depth16To8(refinedDepth).data, sizeof(uchar)*DEPTH_WIDTH*DEPTH_HEIGHT);
}
void TouchDeviceManager::allocateAfterConfigure() {
	//버퍼 변수들 메모리 할당
	if (!depth_refined.isAllocated())
		depth_refined.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);

	if (!depth_refined_8bit.isAllocated())
		depth_refined_8bit.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);

	if (depth2camera_table.size() == 0)
		depth2camera_table.resize(DEPTH_WIDTH * DEPTH_HEIGHT);

	if (all_depth_points.size() == 0) {
		all_depth_points.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
		for (size_t y = 0; y < DEPTH_HEIGHT; y++)
			for (size_t x = 0; x < DEPTH_WIDTH; x++)
				all_depth_points[y*DEPTH_WIDTH + x] = ofPoint(x, y);
	}

	if (depth2projection_table.size() == 0)
		depth2projection_table.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
/*
	if (pointcloud_mesh.getVertices().size() == 0) {
		pointcloud_mesh.setMode(OF_PRIMITIVE_POINTS);
		pointcloud_mesh.getVertices().resize(DEPTH_WIDTH * DEPTH_HEIGHT);
		pointcloud_mesh.getColors().resize(DEPTH_WIDTH * DEPTH_HEIGHT);
		*/
		/*
		오른손 나사 법칙에 맞게 인덱싱
		00 01
		10 11
		-> 00-10-01 / 01-10-11
		*/
		/*
		for (int y = 0; y < DEPTH_HEIGHT - 1; y++) {
			for (int x = 0; x < DEPTH_WIDTH - 1; x++) {
				pointcloud_mesh.addIndex(x + y*DEPTH_WIDTH);               // 00
				pointcloud_mesh.addIndex(x + (y + 1)*DEPTH_WIDTH);         // 10
				pointcloud_mesh.addIndex((x + 1) + y*DEPTH_WIDTH);         // 01

				pointcloud_mesh.addIndex((x + 1) + y*DEPTH_WIDTH);         // 01
				pointcloud_mesh.addIndex(x + (y + 1)*DEPTH_WIDTH);         // 10
				pointcloud_mesh.addIndex((x + 1) + (y + 1)*DEPTH_WIDTH);   // 11
			}
		}
	}

	if (pointcloud_color.size() == 0) pointcloud_color.resize(DEPTH_WIDTH * DEPTH_HEIGHT);

	if (pointcloud_vertex.size() == 0) pointcloud_vertex.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	*/
}


void TouchDeviceManager::calibrateCameras() {
	bool was_flip_horiontal = isFlipHoritonzal();
	bool was_flip_vertical = isFlipVertical();

	setFlipHorizontal(false);
	setFlipVertical(false);

	calibrateDepthCamera();
	depthCalib.setupCalibration();

	calibrateColorCamera();
	colorCalib.setupCalibration();

	setFlipHorizontal(was_flip_horiontal);
	setFlipVertical(was_flip_vertical);
}

void TouchDeviceManager::calibrateDepthCamera() {
	//먼저, SDK의 API를 이용해서 depthCamera 별도로 calibration을 진행
	int width = getDepthWidth(), height = getDepthHeight();

	vector<Point3f> cameraPoints;
	for (float x = -1500; x <= 1500; x += 200)
		for (float y = -1500; y <= 1500; y += 200)
			for (float z = 1500; z <= 4500; z += 500)
				cameraPoints.push_back(Point3f(x, y, z));
	vector<Point2f> imagePoints = XY(cameraToDepth(XYZ(cameraPoints)));

	depthCalib.cameraMatrix = (Mat1d(3, 3) <<
		width, 0, width / 2.,
		0, height, height / 2.,
		0, 0, 1);
	depthCalib.distCoeffs = Mat::zeros(5, 1, CV_64F);
	vector<Mat> camRot;
	vector<Mat> camTran;
	float reprojError =
		cv::calibrateCamera(vector<vector<Point3f>>(1, cameraPoints), vector<vector<Point2f>>(1, imagePoints),
			Size(width, height), depthCalib.cameraMatrix, depthCalib.distCoeffs, camRot, camTran,
			CV_CALIB_USE_INTRINSIC_GUESS, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 1000, DBL_EPSILON));

	depthCalib.resolutionX = width;
	depthCalib.resolutionY = height;
	depthCalib.rotationVec = camRot[0];
	depthCalib.translation = camTran[0];
	depthCalib.reprojError = reprojError;
}

void TouchDeviceManager::calibrateColorCamera() {
	//먼저, SDK의 API를 이용해서 colorCamera 별도로 calibration을 진행
	int width = getColorWidth(), height = getColorHeight();

	vector<Point3f> cameraPoints;
	for (float x = -1500; x <= 1500; x += 200)
		for (float y = -1500; y <= 1500; y += 200)
			for (float z = 1500; z <= 4500; z += 500)
				cameraPoints.push_back(Point3f(x, y, z));
	vector<Point2f> imagePoints = XY(cameraToColor(XYZ(cameraPoints)));

	vector<Point3f>::iterator itr_cam = cameraPoints.begin();
	vector<Point2f>::iterator itr_img = imagePoints.begin();
	while (itr_cam != cameraPoints.end() && itr_img != imagePoints.end()) {
		auto pt = *itr_img;
		if (pt.x >= 0 && pt.x < width && pt.y >= 0 && pt.y < height) {
			itr_cam++;
			itr_img++;
		}
		else {
			itr_cam = cameraPoints.erase(itr_cam);
			itr_img = imagePoints.erase(itr_img);
		}
	}

	colorCalib.cameraMatrix = (Mat1d(3, 3) <<
		width, 0, width / 2.,
		0, height, height / 2.,
		0, 0, 1);
	colorCalib.distCoeffs = Mat::zeros(5, 1, CV_64F);
	vector<Mat> camRot;
	vector<Mat> camTran;
	float reprojError =
		cv::calibrateCamera(vector<vector<Point3f>>(1, cameraPoints), vector<vector<Point2f>>(1, imagePoints),
			Size(width, height), colorCalib.cameraMatrix, colorCalib.distCoeffs, camRot, camTran,
			CV_CALIB_USE_INTRINSIC_GUESS, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 1000, DBL_EPSILON));

	colorCalib.resolutionX = width;
	colorCalib.resolutionY = height;
	colorCalib.rotationVec = camRot[0];
	colorCalib.translation = camTran[0];
	colorCalib.reprojError = reprojError;
}

vector<ofPoint> TouchDeviceManager::cameraToProjection(const vector<ofPoint>& o) {
	vector<cv::Point2f> projected;
	if (projectorCalib.isLoaded())
		projectorCalib.projectPoints(XYZ(o), projected);
	else {
		ofLogFatalError(getDeviceType()) << "cameraToProjection calibration file not loaded!";
		FILE_LINE_FUNCTION;
	}
	if (false) { /*projectPoints 구동방식 검증 코드*/
				 // s(1) * ptProj(3x1) = A(3x3)(R(3x3) * ptWorld(3x1) + t(3x1)) http://darkpgmr.tistory.com/82
				 // scale factor = viewdist/zdist http://stackoverflow.com/questions/2881639/ive-got-my-2d-3d-conversion-working-perfectly-how-to-do-perspective
		cv::Mat mr = cv::Mat::zeros(3, 3, CV_64F);
		Rodrigues(projectorCalib.rotationVec, mr);
		cv::Mat mt = projectorCalib.translation;
		cv::Mat A = projectorCalib.cameraMatrix; // 3x3

		cv::Mat matWorld = cv::Mat::zeros(3, 1, CV_64F);
		((double*)matWorld.data)[0] = o[0].x;
		((double*)matWorld.data)[1] = o[0].y;
		((double*)matWorld.data)[2] = o[0].z;

		cv::Mat matProj = cv::Mat::zeros(3, 1, CV_64F);
		matProj = A * (mr * matWorld + mt);

		double s = ((double*)matProj.data)[2];
		cout << ((double*)matProj.data)[0] / s << " " << ((double*)matProj.data)[1] / s << " " << s << endl;
		cout << projected[0].x << " " << projected[0].y << " " << o[0].z << endl << endl;
	}
	return XY(projected);
}

void Joints::drawJoint3D() {
	ofPushStyle();
	ofSetColor(ofColor::green);
	ofDrawSphere(point_3d, 30);
	ofPopStyle();
}

void Joints::drawJoint2D() {
	ofPushStyle();
	ofSetColor(ofColor::green);
	ofDrawSphere(point_2d, 5);
	ofPopStyle();
}

void Skeletons::drawJoints3D() {
	ofPushStyle();
	ofSetColor(ofColor::white);
	ofSetLineWidth(5);

	ofDrawLine(joints[Head].point_3d, joints[Neck].point_3d);
	ofDrawLine(joints[Neck].point_3d, joints[Body].point_3d);
	ofDrawBitmapString("player:" + to_string(player_index), joints[Head].point_3d);

	ofDrawLine(joints[L_Shoulder].point_3d, joints[R_Shoulder].point_3d);
	ofDrawLine(joints[L_Shoulder].point_3d, joints[L_Elbow].point_3d);
	ofDrawLine(joints[L_Elbow].point_3d, joints[L_Hand].point_3d);
	ofDrawLine(joints[R_Shoulder].point_3d, joints[R_Elbow].point_3d);
	ofDrawLine(joints[R_Elbow].point_3d, joints[R_Hand].point_3d);

	ofDrawLine(joints[L_Shoulder].point_3d, joints[Body].point_3d);
	ofDrawLine(joints[R_Shoulder].point_3d, joints[Body].point_3d);
	ofDrawLine(joints[Body].point_3d, joints[L_Hip].point_3d);
	ofDrawLine(joints[Body].point_3d, joints[R_Hip].point_3d);

	ofDrawLine(joints[L_Hip].point_3d, joints[R_Hip].point_3d);
	ofDrawLine(joints[L_Hip].point_3d, joints[L_Knee].point_3d);
	ofDrawLine(joints[L_Knee].point_3d, joints[L_Foot].point_3d);
	ofDrawLine(joints[R_Hip].point_3d, joints[R_Knee].point_3d);
	ofDrawLine(joints[R_Knee].point_3d, joints[R_Foot].point_3d);

	ofPopStyle();

	for (auto& joint : joints) joint.drawJoint3D();
}

void Skeletons::drawJoints2D() {
	ofPushStyle();
	ofSetColor(ofColor::white);
	ofSetLineWidth(3);

	ofDrawLine(joints[Head].point_2d, joints[Neck].point_2d);
	ofDrawLine(joints[Neck].point_2d, joints[Body].point_2d);
	ofDrawBitmapString("player:" + to_string(player_index), joints[Head].point_2d);

	ofDrawLine(joints[L_Shoulder].point_2d, joints[R_Shoulder].point_2d);
	ofDrawLine(joints[L_Shoulder].point_2d, joints[L_Elbow].point_2d);
	ofDrawLine(joints[L_Elbow].point_2d, joints[L_Hand].point_2d);
	ofDrawLine(joints[R_Shoulder].point_2d, joints[R_Elbow].point_2d);
	ofDrawLine(joints[R_Elbow].point_2d, joints[R_Hand].point_2d);

	ofDrawLine(joints[L_Shoulder].point_2d, joints[Body].point_2d);
	ofDrawLine(joints[R_Shoulder].point_2d, joints[Body].point_2d);
	ofDrawLine(joints[Body].point_2d, joints[L_Hip].point_2d);
	ofDrawLine(joints[Body].point_2d, joints[R_Hip].point_2d);

	ofDrawLine(joints[L_Hip].point_2d, joints[R_Hip].point_2d);
	ofDrawLine(joints[L_Hip].point_2d, joints[L_Knee].point_2d);
	ofDrawLine(joints[L_Knee].point_2d, joints[L_Foot].point_2d);
	ofDrawLine(joints[R_Hip].point_2d, joints[R_Knee].point_2d);
	ofDrawLine(joints[R_Knee].point_2d, joints[R_Foot].point_2d);

	ofPopStyle();

	for (auto& joint : joints) joint.drawJoint2D();
}
