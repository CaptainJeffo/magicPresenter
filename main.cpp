/*
Copyright 2016 Chris Papenfuﬂ

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

//typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType;
bool active = false, detectPlane = false, findScreen = true;

// PCL Visualizer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
	new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

cv::Mat clrImage;

std::string cloudId = "cloud";

bool updated = false, clrImageAvailable = false;
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::ModelCoefficients::ConstPtr preziPlane = nullptr;
float hitPoints[BODY_COUNT][2][3];

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
				hitPoints[bdyIdx][side][0] = joints[hand].Position.X + dist * measureVector[0];
				hitPoints[bdyIdx][side][1] = joints[hand].Position.Y + dist * measureVector[1];
				hitPoints[bdyIdx][side][2] = joints[hand].Position.Z + dist * measureVector[2];
				cout << "HIT:" << std::to_string(bdyIdx) << " SIDE:" << std::to_string(side) << endl;
			}
		}
	}
}

void calculateHitPoints(IBody* curBdy, int bdyIdx) {
	BOOLEAN* status = nullptr;
	HRESULT result(S_OK);
	for (int curHand = 0; curHand < 2; curHand++)
	{
		for (int coord = 0; coord < 3; coord++)
		{
			hitPoints[bdyIdx][curHand][coord] = 0;
		}
	}
	Joint joints[JointType_Count];
	result = curBdy->GetJoints(JointType_Count, joints);
	if (SUCCEEDED(result)) {
		calculateHitPoints(curBdy, bdyIdx, JointType_HandLeft, JointType_ElbowLeft, LEFT, joints);
		calculateHitPoints(curBdy, bdyIdx, JointType_HandRight, JointType_ElbowRight, RIGHT, joints);
	}
}

std::vector<cv::Point2f> scene_corners(4);
std::vector<pcl::PointXYZ> presentCorners(4);
cv::Mat H;


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
				detectPlane = true;
				active = true;
			}
		}
		else if (cb.getKeySym() == "s" && cb.keyDown()) {
			findScreen = !findScreen;
			if (!findScreen)
				for (int cornerIdx = 0; cornerIdx < 4; cornerIdx++)
				{
					presentCorners[cornerIdx] = grabber->getWorldCoordinateFromColor(scene_corners[cornerIdx].x, scene_corners[cornerIdx].y);
					pcl::PointXYZ pnt = presentCorners[cornerIdx];
					
					float tmp = (preziPlane->values[0] * pnt.x) + (preziPlane->values[1] * pnt.y) + (preziPlane->values[2] * pnt.z);
					if (tmp > 0) {//Schnittpunkt existiert.
						float dist = ((-preziPlane->values[3] - preziPlane->values[0] * 0 - preziPlane->values[1] * 0 - preziPlane->values[2] * 0) / tmp);
						if (dist > 0) {
							pnt.x = dist * pnt.x;
							pnt.y = dist * pnt.y;
							pnt.z = dist * pnt.z;

							std::string id = "corner" + std::to_string(cornerIdx);
							if (viewer->contains(id)) {
								viewer->removeShape(id);
							}
							viewer->addCube(pnt.x - .05, pnt.x + .05, pnt.y - .05, pnt.y + .05, pnt.z - .05, pnt.z + .05,
								0.9, 0.9, 1, //RGB
								id);
							viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, id, 0);//R,G,B
							viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, id, 0);
							viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id, 0);
						}
					}
				}
		}
	};
	viewer->registerKeyboardCallback(keyPressedFunc);

	// Point Cloud
	pcl::PointCloud<PointType>::Ptr cloud;

	boost::mutex mutex;
	boost::function<void(const boost::shared_ptr<IBody*[]>&)> functionBdy =
		[&cloud, &mutex](const boost::shared_ptr<IBody*[]>& ptr) {
		if (preziPlane == nullptr) return;
		HRESULT result(S_OK);
		boost::mutex::scoped_lock lock(mutex);
		for (int bdyIdx = 0; bdyIdx < BODY_COUNT; bdyIdx++)
		{
			IBody* curBdy = ptr[bdyIdx];
			//IBody* curBdy = (*ptr)[bdyIdx];
			if (curBdy == NULL) { continue; }
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
		[&cloud, &mutex](const boost::shared_ptr<cv::Mat>& ptr) {
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

			if (detectPlane) {
				preziPlane = findCluster_PlaneSegmentation(ptr, inliers);
				detectPlane = false;

				if (preziPlane->values.size() == 4) {
					if (viewer->contains("plane")) {
						viewer->removeShape("plane", 0);
					}
					viewer->addPlane(*preziPlane, "plane", 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "plane", 0);//R,G,B
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane", 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane", 0);

				}
				else {
					detectPlane = true;
				}
			}

			//draw pixel from cloud within plane
			/*pcl::PointCloud<PointType>::Ptr planeCloud(new pcl::PointCloud<PointType>());
			pcl::copyPointCloud(*ptr, *inliers, *planeCloud);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planeCloud, 122, 122, 122);
			if (!viewer->updatePointCloud(planeCloud, single_color, cloudId + "_plane")) {
				viewer->addPointCloud(planeCloud, single_color, cloudId + "_plane");
			}*/
			for (int i = 0; i < BODY_COUNT; i++)
			{
				for (int handIdx = 0; handIdx < 2; handIdx++)
				{
					if (viewer->contains("cube" + std::to_string(i) + "_" + std::to_string(handIdx))) {
						viewer->removeShape("cube" + std::to_string(i) + "_" + std::to_string(handIdx));
					}
					if (hitPoints[i][handIdx][0] == 0
						&& hitPoints[i][handIdx][1] == 0
						&& hitPoints[i][handIdx][2] == 0) {
						continue;
					}
					if (hitPoints[i][handIdx][1] < -1 || hitPoints[i][handIdx][1] > 4) { continue; }

					viewer->addCube(hitPoints[i][handIdx][0] - .05, hitPoints[i][handIdx][0] + .05, hitPoints[i][handIdx][1] - .05, hitPoints[i][handIdx][1] + .05, hitPoints[i][handIdx][2] - .05, hitPoints[i][handIdx][2] + .05,
						0.9, 0.9, 1, //RGB
						"cube" + std::to_string(i) + "_" + std::to_string(handIdx));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.1, "cube" + std::to_string(i) + "_" + std::to_string(handIdx), 0);//R,G,B
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "cube" + std::to_string(i) + "_" + std::to_string(handIdx), 0);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube" + std::to_string(i) + "_" + std::to_string(handIdx), 0);
				}
			}
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(ptr, 0, 255, 0);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*ptr, *ptr, indices);
		if (ptr->size() > 100 && !viewer->updatePointCloud(ptr, single_color, cloudId)) {
			viewer->addPointCloud(ptr, single_color, cloudId);
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
	viewer->addCoordinateSystem(3., 0, 0, 10, 1);

	cv::Mat processImg = getImage();
	cv::flip(processImg, processImg, 1);

	processImg.convertTo(processImg, CV_8UC4);
	cv::imshow("Process", processImg);

	while (!viewer->wasStopped()) {
		boost::mutex::scoped_try_lock lock(mutex);
		if (updated && lock.owns_lock() && clrImageAvailable) {
			updated = false;
			cv::imshow("Color", clrImage);

			if (findScreen) {
				H = findImage(clrImage, &scene_corners);

			}
			else {
				//std::vector<cv::Point2f> input(1), output(1);
				//input[0] = cvPoint(0, 0);
				//perspectiveTransform(input, output, -H);
			}

			viewer->spinOnce();
		}
	}

	// Stop Grabber
	grabber->stop();

	return 0;
}