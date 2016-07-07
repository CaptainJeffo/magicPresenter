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
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include <limits.h>


namespace pcl
{
	struct pcl::PointXYZ;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease(Interface *& IRelease)
	{
		if (IRelease != NULL) {
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class Kinect2Grabber : public pcl::Grabber
	{
	public:
		Kinect2Grabber();
		virtual ~Kinect2Grabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;
		virtual pcl::PointXYZ getWorldCoordinateFromColor(float x, float y);
		virtual pcl::PointXYZ getWorldCoordinateFromColor_int(int x, int y);
		float clrImageScale = 2.0;

		typedef void (signal_Kinect2_PointXYZ)(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_Kinect2_Body)(const boost::shared_ptr<IBody*[]>&);
		typedef void (signal_Kinect2_Color)(const boost::shared_ptr<cv::Mat>&);
		typedef void (signal_Range_Def)(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&);

	protected:
		boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_Kinect2_Body>* signal_Body;
		boost::signals2::signal<signal_Kinect2_Color>* signal_Color;
		boost::signals2::signal<signal_Range_Def>* signal_Range;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMin, cloudMax;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(UINT16* depthBuffer);

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;

		HRESULT result;
		IKinectSensor* sensor;
		IMultiSourceFrameReader* multiReader;

		ICoordinateMapper* mapper;
		IDepthFrameSource* depthSource;
		IDepthFrameReader* depthReader;
		IColorFrameSource* colorSource;
		IColorFrameReader* colorReader;
		IBodyFrameSource* bodySource;
		IBodyFrameReader* bodyReader;


		CameraSpacePoint* cameraSpacePnt;

		int depthWidth;
		int depthHeight;
		int clrWidth;
		int clrHeight;

		std::vector<UINT16> depthBuffer;
		IBody* bodyData[BODY_COUNT] = { 0 };

		unsigned int clrBufferSize;
		cv::Mat bufferMat;
		cv::Mat colorMat;

	};

	pcl::Kinect2Grabber::Kinect2Grabber()
		: sensor(nullptr)
		, mapper(nullptr)
		, depthSource(nullptr)
		, depthReader(nullptr)
		, colorSource(nullptr)
		, colorReader(nullptr)
		, bodySource(nullptr)
		, bodyReader(nullptr)
		, cameraSpacePnt(NULL)
		, multiReader(nullptr)
		, result(S_OK)
		, depthWidth(512)
		, depthHeight(424)
		, clrWidth(1900)
		, clrHeight(1200)
		, clrBufferSize()
		, depthBuffer()
		, running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_Body(nullptr)
		, signal_Color(nullptr)
		, signal_Range(nullptr)
		, cloudMin(new pcl::PointCloud<pcl::PointXYZ>())
		, cloudMax(new pcl::PointCloud<pcl::PointXYZ>())
	{
		// Create Sensor Instance
		result = GetDefaultKinectSensor(&sensor);
		if (FAILED(result)) {
			throw std::exception("Exception : GetDefaultKinectSensor()");
		}

		// Open Sensor
		result = sensor->Open();
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::Open()");
		}

		// Retrieved Coordinate Mapper
		result = sensor->get_CoordinateMapper(&mapper);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
		}

		// Retrieved Depth Frame Source
		result = sensor->get_DepthFrameSource(&depthSource);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
		}

		// Retrive Color Frame Source
		result = sensor->get_ColorFrameSource(&colorSource);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
		}

		// Retrive Body Frame Source
		result = sensor->get_BodyFrameSource(&bodySource);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_BodyFrameSource()");
		}

		// Retrieved Color Frame Size
		IFrameDescription* clrDescription;
		result = colorSource->get_FrameDescription(&clrDescription);
		if (FAILED(result)) {
			throw std::exception("Exception : IColorFrameSource::get_FrameDescription()");
		}

		result = clrDescription->get_Width(&clrWidth);
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = clrDescription->get_Height(&clrHeight);
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}

		cameraSpacePnt = new CameraSpacePoint[clrWidth * clrHeight];
		clrBufferSize = clrWidth * clrHeight * 4 * sizeof(unsigned char);
		SafeRelease(clrDescription);

		// Retrieved Depth Frame Size
		IFrameDescription* depthDescription;
		result = depthSource->get_FrameDescription(&depthDescription);
		if (FAILED(result)) {
			throw std::exception("Exception : IDepthFrameSource::get_FrameDescription()");
		}

		result = depthDescription->get_Width(&depthWidth); // 512
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = depthDescription->get_Height(&depthHeight); // 424
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}

		SafeRelease(depthDescription);

		// To Reserve Depth Frame Buffer
		depthBuffer.resize(depthWidth * depthHeight);
		bufferMat.create(clrHeight, clrWidth, CV_8UC4);
		colorMat.create(clrHeight / clrImageScale, clrWidth / clrImageScale, CV_8UC4);

		signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
		signal_Body = createSignal<signal_Kinect2_Body>();
		signal_Color = createSignal<signal_Kinect2_Color>();
		signal_Range = createSignal<signal_Range_Def>();

		cloudMin->points.resize(1);
		cloudMax->points.resize(1);
	}

	pcl::Kinect2Grabber::~Kinect2Grabber() throw()
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();
		disconnect_all_slots<signal_Kinect2_Body>();
		disconnect_all_slots<signal_Kinect2_Color>();
		disconnect_all_slots<signal_Range_Def>();

		thread.join();

		// End Processing
		if (sensor) {
			sensor->Close();
		}
		SafeRelease(sensor);
		SafeRelease(mapper);
		SafeRelease(depthSource);
		SafeRelease(depthReader);
		SafeRelease(colorSource);
		SafeRelease(colorReader);
		SafeRelease(bodySource);
		SafeRelease(bodyReader);
		SafeRelease(multiReader);
	}

	void pcl::Kinect2Grabber::start()
	{
		result = sensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body, &multiReader);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::OpenMultiSourceFrameReader()");
		}

		running = true;

		thread = boost::thread(&Kinect2Grabber::threadFunction, this);
	}

	void pcl::Kinect2Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::Kinect2Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::Kinect2Grabber::getName() const
	{
		return std::string("Kinect2Grabber");
	}

	float pcl::Kinect2Grabber::getFramesPerSecond() const
	{
		return 30.0f;
	}

	void getColorData(IMultiSourceFrame* multiFrame, UINT cap, cv::Mat* bufferData, cv::Mat* clrData, float imgScale) {
		HRESULT result(S_OK);
		IColorFrameReference* clrFrameRef = nullptr;
		IColorFrame* clrFrame = nullptr;
		result = multiFrame->get_ColorFrameReference(&clrFrameRef);
		if (SUCCEEDED(result)) {
			result = clrFrameRef->AcquireFrame(&clrFrame);
			if (SUCCEEDED(result)) {
				result = clrFrame->CopyConvertedFrameDataToArray(cap, bufferData->data, ColorImageFormat::ColorImageFormat_Bgra);
				if (FAILED(result)) {
					throw std::exception("Exception : IColorFrame::CopyConvertedFrameDataToArray()");
				}
				cv::resize(*bufferData, *clrData, cv::Size(), 1.0 / imgScale, 1.0 / imgScale);
				//cv::flip(*clrData, *clrData, 1);
			}
			SafeRelease(clrFrame);
		}
		SafeRelease(clrFrameRef);
	}

	void getBodyData(IMultiSourceFrame* multiFrame, UINT cap, IBody** bodyData) {
		HRESULT result(S_OK);
		IBodyFrameReference* bodyFrameRef = nullptr;
		IBodyFrame* bodyFrame = nullptr;
		result = multiFrame->get_BodyFrameReference(&bodyFrameRef);
		if (SUCCEEDED(result)) {
			result = bodyFrameRef->AcquireFrame(&bodyFrame);
			if (SUCCEEDED(result)) {
				result = bodyFrame->GetAndRefreshBodyData(cap, bodyData);
				if (FAILED(result)) {
					throw std::exception("Exception : IBodyFrame::GetAndRefreshBodyData()");
				}
			}
			SafeRelease(bodyFrame);
		}
		SafeRelease(bodyFrameRef);
	}

	void getDepthFrame(IMultiSourceFrame* multiFrame, UINT cap, UINT16* frameData) {
		HRESULT result(S_OK);
		IDepthFrameReference* depthFrameRef = nullptr;
		IDepthFrame* depthFrame = nullptr;
		result = multiFrame->get_DepthFrameReference(&depthFrameRef);
		if (SUCCEEDED(result)) {
			result = depthFrameRef->AcquireFrame(&depthFrame);
			if (SUCCEEDED(result)) {
				result = depthFrame->CopyFrameDataToArray(cap, frameData);
				if (FAILED(result)) {
					throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
				}
			}
			SafeRelease(depthFrame);
		}
		SafeRelease(depthFrameRef);
	}

	void pcl::Kinect2Grabber::threadFunction()
	{
		while (!quit) {
			boost::unique_lock<boost::mutex> lock(mutex);

			// Acquire Latest Depth Frame
			IMultiSourceFrame* multiFrame = nullptr;
			result = multiReader->AcquireLatestFrame(&multiFrame);
			if (SUCCEEDED(result)) {
				getDepthFrame(multiFrame, depthBuffer.size(), &depthBuffer[0]);
				result = mapper->MapColorFrameToCameraSpace(depthWidth*depthHeight, &depthBuffer[0], clrWidth*clrHeight, cameraSpacePnt);
				if (FAILED(result)) {
					throw std::exception("Exception : MapColorFrameToCameraSpace");
				}
				getBodyData(multiFrame, BODY_COUNT, bodyData);
				getColorData(multiFrame, clrBufferSize, &bufferMat, &colorMat, clrImageScale);
			}
			SafeRelease(multiFrame);

			lock.unlock();

			if (signal_PointXYZ->num_slots() > 0) {
				signal_PointXYZ->operator()(convertDepthToPointXYZ(&depthBuffer[0]));
			}
			if (signal_Body->num_slots() > 0) {
				boost::shared_ptr<IBody*[]> bodies = boost::make_shared<IBody*[]>(BODY_COUNT);
				for (int bdyIdx = 0; bdyIdx < BODY_COUNT; bdyIdx++)
				{
					bodies[bdyIdx] = bodyData[bdyIdx];
				}
				signal_Body->operator()(bodies);
			}
			if (signal_Color->num_slots() > 0) {
				signal_Color->operator()(boost::make_shared<cv::Mat>(colorMat));
			}

		}
	}

	pcl::PointXYZ pcl::Kinect2Grabber::getWorldCoordinateFromColor(float xClr, float yClr) {
		xClr = xClr * clrImageScale;
		yClr = yClr * clrImageScale;
		std::vector<pcl::PointXYZ> pnts(9);
		float idx = 0.;
		for (int x = -1; x < 2; x++)
		{
			for (int y = -1; y < 2; y++)
			{
				pnts[idx++] = getWorldCoordinateFromColor_int(xClr + x, yClr + y);
			}
		}
		idx = 0.;
		float tmpX = 0., tmpY = 0., tmpZ = 0., idx2 = 0., idx3 = 0.;
		pcl::PointXYZ ret;
		for each (pcl::PointXYZ pnt in pnts)
		{
			if (pnt.x > -20. && pnt.x < 20.) {//not NAN
				tmpX = tmpX + pnt.x;
				idx++;
			}
			if (pnt.y > -20. && pnt.y < 20.) {//not NAN
				tmpY = tmpY + pnt.y;
				idx2++;
			}
			if (pnt.z > -20. && pnt.z < 20.) {//not NAN
				tmpZ = tmpZ + pnt.z;
				idx3++;
			}
		}
		ret.x = tmpX / idx;
		ret.y = tmpY / idx2;
		ret.z = tmpZ / idx3;
		return ret;
	}

	pcl::PointXYZ pcl::Kinect2Grabber::getWorldCoordinateFromColor_int(int x, int y) {
		int clrIdx = y * clrWidth + x;
		if (clrIdx < 0 || clrIdx > clrWidth * clrHeight) return pcl::PointXYZ(0., 0., 0.);
		CameraSpacePoint camSpacePoint = cameraSpacePnt[clrIdx];
		pcl::PointXYZ pnt = { camSpacePoint.X, camSpacePoint.Y, camSpacePoint.Z };
		return pnt;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_p->width = static_cast<uint32_t>(depthWidth);
		cloud_p->height = static_cast<uint32_t>(depthHeight);
		cloud_p->is_dense = false;

		cloud_p->points.resize(cloud_p->height * cloud_p->width);

		if (signal_Range->num_slots() > 0) {
			signal_Range->operator()(cloudMin, cloudMax);
		}

		pcl::PointXYZ* pt = &cloud_p->points[0];
		int validCnt = 0;
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++) {
				//pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ

				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);

				bool validx = (cloudMin->points[0].x == cloudMax->points[0].x
					|| (cloudMin->points[0].x <= cameraSpacePoint.X && cloudMax->points[0].x >= cameraSpacePoint.X));
				bool validy = (cloudMin->points[0].y == cloudMax->points[0].y
					|| (cloudMin->points[0].y <= cameraSpacePoint.Y && cloudMax->points[0].y >= cameraSpacePoint.Y));
				bool validz = (cloudMin->points[0].z == cloudMax->points[0].z
					|| (cloudMin->points[0].z <= cameraSpacePoint.Z && cloudMax->points[0].z >= cameraSpacePoint.Z));

				bool validPoint = validx && validy && validz;
				if (validPoint) {
					pt->x = cameraSpacePoint.X;
					pt->y = cameraSpacePoint.Y;
					pt->z = cameraSpacePoint.Z;
					pt++;
					validCnt++;
				}

			}
		}
		cloud_p->points.resize(validCnt);
		return cloud_p;
	}

}

