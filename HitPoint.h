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

#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace cefoot
{
	class HitPoint 
	{
	public:
		virtual bool isObsolete(time_t curTime);
		Eigen::Vector3f point;
		time_t creationTime;

	protected:

	};
	bool cefoot::HitPoint::isObsolete(time_t curTime)
	{
		return curTime - creationTime > 2;
	}

}