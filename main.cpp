/*
Copyright 2016 Chris Papenfuß

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <iostream>


#include "kinect_pcl_grabber.h"
#include "detection-utils.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include "image-detection.h"

#define LEFT	0
#define RIGHT	1
enum DebugInfo
{
	None = 0,
	PointCloud = 0x1,
	OpenCV = 0x2,
	HitPoints = 0x4,
	Corners = 0x8
};
//typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType;
bool active = false, detectingPlane = true, findScreen = false, planeDetected = false;

// PCL Visualizer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
	new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

cv::Mat clrImage;

std::string cloudId = "cloud";

bool updated = false, clrImageAvailable = false;

pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::ModelCoefficients::ConstPtr preziPlane;
Eigen::Vector3f hitPoints[1/*BODY_COUNT*/][4];

void calculateHitPoints(IBody* curBdy, int bdyIdx, JointType hand, JointType elbow, int side, Joint* joints) {
	static float measureVector[] = { 0., 0., 0. };
	if (joints[hand].TrackingState > TrackingState::TrackingState_NotTracked && joints[elbow].TrackingState > TrackingState::TrackingState_NotTracked) {
		measureVector[0] = joints[hand].Position.X - joints[elbow].Position.X;
		measureVector[1] = joints[hand].Position.Y - joints[elbow].Position.Y;
		measureVector[2] = joints[hand].Position.Z - joints[elbow].Position.Z;

		float tmp = (preziPlane->values[0] * measureVector[0]) + (preziPlane->values[1] * measureVector[1]) + (preziPlane->values[2] * measureVector[2]);
		if (tmp > 0) {//Schnittpunkt existiert.
			float dist = ((-preziPlane->values[3] - preziPlane->values[0] * joints[hand].Position.X - preziPlane->values[1] * joints[hand].Position.Y - preziPlane->values[2] * joints[hand].Position.Z) / tmp);
			if (dist > 0) {
				hitPoints[bdyIdx][side] = Eigen::Vector3f(
					joints[hand].Position.X + dist * measureVector[0],
					joints[hand].Position.Y + dist * measureVector[1],
					joints[hand].Position.Z + dist * measureVector[2]);
			}
		}
	}
}

void calculateHitPoints(IBody* curBdy, int bdyIdx) {
	BOOLEAN* status = nullptr;
	HRESULT result(S_OK);
	Joint joints[JointType_Count];
	result = curBdy->GetJoints(JointType_Count, joints);
	if (SUCCEEDED(result)) {
		//calculateHitPoints(curBdy, bdyIdx, JointType_HandLeft, JointType_Head, LEFT, joints);
		//calculateHitPoints(curBdy, bdyIdx, JointType_HandRight, JointType_Head, RIGHT, joints);
		calculateHitPoints(curBdy, bdyIdx, JointType_HandLeft, JointType_ElbowLeft, LEFT, joints);
		calculateHitPoints(curBdy, bdyIdx, JointType_HandRight, JointType_ElbowRight, RIGHT, joints);
	}
}

std::vector<cv::Point2f> scene_corners(4);
std::vector<pcl::PointXYZ> presentCorners = {
	{ 0,0,0 } ,
	{ 0,0,0 } ,
	{ 0,0,0 } ,
	{ 0,0,0 } };//(4);
cv::Mat H;

void DetectPlane(pcl::PointCloud<PointType>::Ptr ptr) {
	preziPlane = findCluster_PlaneSegmentation(ptr, inliers);

	if (preziPlane->values.size() == 4) {
		if (viewer->contains("plane")) {
			viewer->removeShape("plane", 0);
		}
		viewer->addPlane(*preziPlane, "plane", 0);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "plane", 0);//R,G,B
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane", 0);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane", 0);
		planeDetected = true;
	}
	else {
		detectingPlane = true;
	}
}

void MarkHitPoints() {
	if (planeDetected) {
		for (int bdyIdx = 0; bdyIdx < BODY_COUNT; bdyIdx++)
		{
			for (int handIdx = 0; handIdx < 2; handIdx++)
			{
				std::string id = "hitPnt" + std::to_string(bdyIdx) + "_" + std::to_string(handIdx);
				if (hitPoints[bdyIdx][handIdx][0] == 0
					&& hitPoints[bdyIdx][handIdx][1] == 0
					&& hitPoints[bdyIdx][handIdx][2] == 0) {
					continue;
				}
				//if (hitPoints[i][handIdx][0] < -1 || hitPoints[i][handIdx][1] > 4) { continue; }

				/*Eigen::Affine3f pose = (Eigen::Affine3f)Eigen::Translation3f(
				hitPoints[bdyIdx][handIdx][0] - hitPoints[bdyIdx][handIdx + 2][0],
				hitPoints[bdyIdx][handIdx][1] - hitPoints[bdyIdx][handIdx + 2][1],
				hitPoints[bdyIdx][handIdx][2] - hitPoints[bdyIdx][handIdx + 2][2]);

				hitPoints[bdyIdx][handIdx + 2] = Eigen::Vector3f(
				hitPoints[bdyIdx][handIdx][0],
				hitPoints[bdyIdx][handIdx][1],
				hitPoints[bdyIdx][handIdx][2]);
				if (!viewer->updateShapePose(id, pose)) {
				viewer->addCube(hitPoints[bdyIdx][handIdx][0] - .05, hitPoints[bdyIdx][handIdx][0] + .05,
				hitPoints[bdyIdx][handIdx][1] - .05, hitPoints[bdyIdx][handIdx][1] + .05,
				hitPoints[bdyIdx][handIdx][2] - .05, hitPoints[bdyIdx][handIdx][2] + .05,
				0.2, 0.2, 0.9, //RGB
				id, 0);
				}*/
				if (viewer->contains(id)) {
					viewer->removeShape(id, 0);
				}
				viewer->addCube(hitPoints[bdyIdx][handIdx][0] - .05, hitPoints[bdyIdx][handIdx][0] + .05,
					hitPoints[bdyIdx][handIdx][1] - .05, hitPoints[bdyIdx][handIdx][1] + .05,
					hitPoints[bdyIdx][handIdx][2] - .05, hitPoints[bdyIdx][handIdx][2] + .05,
					0.2, 0.2, 0.9, //RGB
					id, 0);

				//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.1, id, 0);//R,G,B
				//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, id, 0);
				//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id, 0);
			}
		}
	}
}

void FindScreen(boost::shared_ptr<pcl::Kinect2Grabber> grabber) {
	if (findScreen) {
		H = findImage(clrImage, &scene_corners);
		std::vector<cv::KeyPoint> pnts(4);
		for (int i = 0; i < 4; i++)
		{
			cv::Point2f corner = scene_corners[i];
			pnts[i] = cv::KeyPoint(corner.x, corner.y, 5.);
			presentCorners[i] = grabber->getWorldCoordinateFromColor_int(scene_corners[i].x * grabber->clrImageScale, scene_corners[i].y * grabber->clrImageScale);
		}
		cv::Mat clrImageC;
		clrImage.copyTo(clrImageC);
		try {
			cv::drawKeypoints(clrImageC, pnts, clrImageC, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			cv::imshow("Color", clrImageC);
		}
		catch (cv::Exception e) {
			e.formatMessage();
		}
		for (int cornerIdx = 0; cornerIdx < 4; cornerIdx++)
		{
			pcl::PointXYZ pnt = presentCorners[cornerIdx];
			std::string id = "corner" + std::to_string(cornerIdx);
			if (viewer->contains(id)) {
				viewer->removeShape(id, 0);
			}
			viewer->addCube(pnt.x - .05, pnt.x + .05, pnt.y - .05, pnt.y + .05, pnt.z - .05, pnt.z + .05,
				0.9, 0.9, 1, //RGB
				id, 0);
			//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, id, 0);//R,G,B
			//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, id, 0);
			//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id, 0);
		}
	}
	else {
		cv::imshow("Color", clrImage);
		//std::vector<cv::Point2f> input(1), output(1);
		//input[0] = cvPoint(0, 0);
		//perspectiveTransform(input, output, -H);
	}
}

int main(int argc, char* argv[])
{
	// Kinect2Grabber
	boost::shared_ptr<pcl::Kinect2Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
	boost::function<void(const pcl::visualization::KeyboardEvent&)> keyPressedFunc =
		[&grabber](const pcl::visualization::KeyboardEvent& cb) {
		if (cb.getKeySym() == "a" && cb.keyDown()) {
			initClustering_PlaneSegmentation();
			if (active) {
				active = false;
				if (viewer->contains("cube")) {
					viewer->removeShape("cube");
				}
				if (viewer->contains("plane")) {
					viewer->removeShape("plane");
				}
				if (viewer->contains(cloudId + "_plane")) {
					viewer->removePointCloud(cloudId + "_plane");
				}
			}
			else {
				planeDetected = false;
				detectingPlane = true;
				active = true;
			}
		}
		else if (cb.getKeySym() == "s" && cb.keyDown()) {
			findScreen = !findScreen;
			if (!findScreen && !planeDetected) {
				findScreen = !findScreen;
				std::cout << "Let's detect the plane first ;)" << std::endl;
			}
			if (!findScreen)
				for (int cornerIdx = 0; cornerIdx < 4; cornerIdx++)
				{
					if (scene_corners[cornerIdx].x == 0 && scene_corners[cornerIdx].y == 0) continue;
					//get smoothed values
					presentCorners[cornerIdx] = grabber->getWorldCoordinateFromColor(scene_corners[cornerIdx].x, scene_corners[cornerIdx].y);
					//only on plane
					float tmp = (preziPlane->values[0] * presentCorners[cornerIdx].x) + (preziPlane->values[1] * presentCorners[cornerIdx].y) + (preziPlane->values[2] * presentCorners[cornerIdx].z);
					if (tmp > 0) {//Schnittpunkt existiert.
						float dist = ((-preziPlane->values[3]) / tmp);
						if (dist > 0) {
							presentCorners[cornerIdx].x = presentCorners[cornerIdx].x * dist;
							presentCorners[cornerIdx].y = presentCorners[cornerIdx].y * dist;
							presentCorners[cornerIdx].z = presentCorners[cornerIdx].z * dist;
						}
					}
					pcl::PointXYZ pnt = presentCorners[cornerIdx];
					std::string id = "corner" + std::to_string(cornerIdx);
					if (viewer->contains(id)) {
						viewer->removeShape(id, 0);
					}
					viewer->addCube(pnt.x - .05, pnt.x + .05, pnt.y - .05, pnt.y + .05, pnt.z - .05, pnt.z + .05,
						0.9, 0.9, 1, //RGB
						id, 0);
					//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, id, 0);//R,G,B
					//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, id, 0);
					//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id, 0);
				}
		}
	};
	viewer->registerKeyboardCallback(keyPressedFunc);

	// Point Cloud
	pcl::PointCloud<PointType>::Ptr cloud;

	boost::mutex mutex;
	boost::function<void(const boost::shared_ptr<IBody*[]>&)> functionBdy =
		[&cloud, &mutex](const boost::shared_ptr<IBody*[]>& ptr) {
		if (!planeDetected) return;
		HRESULT result(S_OK);
		boost::mutex::scoped_lock lock(mutex);
		for (int bdyIdx = 0; bdyIdx < BODY_COUNT; bdyIdx++)
		{
			IBody* curBdy = ptr[bdyIdx];
			if (curBdy == NULL) { continue; }
			BOOLEAN tracked = false;
			result = curBdy->get_IsTracked(&tracked);
			if (FAILED(result)
				||
				!tracked) {
				continue;
			}
			calculateHitPoints(curBdy, bdyIdx);
		}
	};


	boost::function<void(pcl::PointCloud<PointType>::Ptr&, pcl::PointCloud<PointType>::Ptr&)> funcRange =
		[](pcl::PointCloud<PointType>::Ptr& minRange, pcl::PointCloud<PointType>::Ptr& maxRange) {
		/*########### FILTER VALUES ###############*/
		minRange->points[0].x = 0;
		maxRange->points[0].x = 0;
		minRange->points[0].z = 0.;
		maxRange->points[0].z = 0.;
		//minRange->points[0].y = -.3;
		//maxRange->points[0].y = .0;
		/*########### END FILTER VALUES ###############*/
	};


	boost::function<void(const boost::shared_ptr<cv::Mat>&)> functionClr =
		[&cloud, &mutex, &grabber](const boost::shared_ptr<cv::Mat>& ptr) {
		boost::mutex::scoped_lock lock(mutex);
		ptr->copyTo(clrImage);
		clrImageAvailable = true;
	};

	// Retrieved Point Cloud Callback Function
	boost::function<void(const pcl::PointCloud<PointType>::Ptr&)> function =
		[&cloud, &mutex, &grabber](const pcl::PointCloud<PointType>::Ptr& ptr) {
		if (ptr->points.size() == 0) return;
		boost::mutex::scoped_lock lock(mutex);
		//cloud = ptr;
		if (active) {
			if (detectingPlane) {
				detectingPlane = false;
				clrImageAvailable = false; //das aktuelle Bild überspringen
				boost::thread t(boost::bind(&DetectPlane, boost::ref(ptr)));
			}

			//draw pixel from cloud within plane
			/*pcl::PointCloud<PointType>::Ptr planeCloud(new pcl::PointCloud<PointType>());
			pcl::copyPointCloud(*ptr, *inliers, *planeCloud);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planeCloud, 122, 122, 122);
			if (!viewer->updatePointCloud(planeCloud, single_color, cloudId + "_plane")) {
				viewer->addPointCloud(planeCloud, single_color, cloudId + "_plane");
			}*/
		}
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(ptr, 0, 255, 0);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*ptr, *ptr, indices);
		if (ptr->size() > 100 && !viewer->updatePointCloud(ptr, single_color, cloudId)) {
			viewer->addPointCloud(ptr, single_color, cloudId, 0);
		}
		updated = true;
	};



	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);
	boost::signals2::connection clrConnection = grabber->registerCallback(functionClr);
	boost::signals2::connection bdyConnection = grabber->registerCallback(functionBdy);
	boost::signals2::connection connectionRange = grabber->registerCallback(funcRange);

	// Start Grabber
	grabber->start();
	viewer->addCoordinateSystem(3., 0, 0, 10, "coords", 0);

	cv::Mat processImg = getImage();
	cv::flip(processImg, processImg, 1);

	processImg.convertTo(processImg, CV_8UC4);
	cv::imshow("Process", processImg);

	while (!viewer->wasStopped()) {
		boost::mutex::scoped_try_lock lock(mutex);
		if (updated && lock.owns_lock() && clrImageAvailable) {
			updated = false;
			FindScreen(grabber);
			MarkHitPoints();
			viewer->spinOnce();
		}
	}

	// Stop Grabber
	grabber->stop();

	return 0;
}