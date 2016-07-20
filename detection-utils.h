#pragma once
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

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

typedef pcl::PointXYZ PointType;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


#define Plane
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/angles.h>
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

const pcl::ModelCoefficients::Ptr oldCoefficients(new pcl::ModelCoefficients);
const pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

void initClustering_PlaneSegmentation() {
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//senkrecht auf definierter Achse
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.005);//max Mittlere Abstand der Punkte zur Ebene
	seg.setAxis(Eigen::Vector3f(0, 0, 1));//definierte Achse
	seg.setEpsAngle(pcl::deg2rad(90.));//Winkel in dem die Ebene liegen kann
}

pcl::ModelCoefficients::ConstPtr findCluster_PlaneSegmentation(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointIndices::Ptr inliers) {
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	//Bei Erfolg enthält 'coefficients' die vier Parameter für die Koordinatenform der Ebene
	//'inliers' enthält die Pixel der Punktwolke, die zu der Ebene gehören
	if (inliers->indices.size() > 0) {
		//std::memcpy(oldCoefficients.get(), coefficients.get(), sizeof(coefficients));
		*oldCoefficients = *coefficients;
	}
	return oldCoefficients;
}



