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
#define NOMINMAX
#define RGB(r,g,b)      ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))
#include <algorithm>
namespace Gdiplus
{
	using std::min;
	using std::max;
};
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "screen/ScreenImage.h"
#include "opencv2/nonfree/nonfree.hpp"
HWND g_HWND = NULL;
BOOL CALLBACK EnumWindowsProcMy(HWND hwnd, LPARAM lParam)
{
	DWORD lpdwProcessId;
	GetWindowThreadProcessId(hwnd, &lpdwProcessId);
	if (lpdwProcessId == lParam)
	{
		g_HWND = hwnd;
		return FALSE;
	}
	return TRUE;
}

struct ENUM_DISP_ARG
{
	TCHAR msg[500];
	int monId;
};

RECT monitor;

// callback function called by EnumDisplayMonitors for each enabled monitor
BOOL CALLBACK EnumDispProc(HMONITOR hMon, HDC dcMon, RECT* pRcMon, LPARAM lParam)
{
	ENUM_DISP_ARG* pArg = reinterpret_cast<ENUM_DISP_ARG*>(lParam);

	//printf("Monitor %d: %d x %d @ %d,%d\r\n", pArg->monId, pRcMon->right - pRcMon->left, pRcMon->bottom - pRcMon->top, pRcMon->left, pRcMon->top);
	pArg->monId++;
	monitor = *pRcMon;

	return TRUE;
}



cv::Mat getImage() {

	ENUM_DISP_ARG arg = { 0 };
	arg.monId = 2;
	EnumDisplayMonitors(0, 0, EnumDispProc, reinterpret_cast<LPARAM>(&arg));
	EnumWindows(EnumWindowsProcMy, 1248);
	HDC hwindowDC, hwindowCompatibleDC;

	int height, width, srcheight, srcwidth;
	HBITMAP hbwindow;
	cv::Mat src;
	BITMAPINFOHEADER  bi;

	hwindowDC = GetDC(g_HWND);
	hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
	SetStretchBltMode(hwindowCompatibleDC, COLORONCOLOR);

	RECT windowsize;    // get the height and width of the screen
	GetClientRect(g_HWND, &windowsize);

	srcheight = windowsize.bottom;
	srcwidth = windowsize.right;
	height = windowsize.bottom / 4;  //change this to whatever size you want to resize to
	width = windowsize.right / 4;

	src.create(height, width, CV_8UC4);

	// create a bitmap
	hbwindow = CreateCompatibleBitmap(hwindowDC, width, height);
	bi.biSize = sizeof(BITMAPINFOHEADER);    //http://msdn.microsoft.com/en-us/library/windows/window/dd183402%28v=vs.85%29.aspx
	bi.biWidth = width;
	bi.biHeight = -height;  //this is the line that makes it draw upside down or not
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	// use the previously created device context with the bitmap
	SelectObject(hwindowCompatibleDC, hbwindow);
	// copy from the window device context to the bitmap device context
	StretchBlt(hwindowCompatibleDC, 0, 0, width, height, hwindowDC, 0, 0, srcwidth, srcheight, SRCCOPY); //change SRCCOPY to NOTSRCCOPY for wacky colors !
	GetDIBits(hwindowCompatibleDC, hbwindow, 0, height, src.data, (BITMAPINFO *)&bi, DIB_RGB_COLORS);  //copy from hwindowCompatibleDC to hbwindow

																									   // avoid memory leak
	DeleteObject(hbwindow); DeleteDC(hwindowCompatibleDC); ReleaseDC(g_HWND, hwindowDC);

	return src;
}


cv::FlannBasedMatcher matcher;

cv::SurfDescriptorExtractor extractor;
double max_dist = 0; double min_dist = 100;
int minHessian = 400;

cv::SurfFeatureDetector detector(minHessian);
cv::Mat progImage;

std::vector<cv::Point2f> obj_corners(4);

cv::Mat descriptors_object, descriptors_scene;
std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
std::vector<cv::DMatch> matches;

cv::Mat findImage(cv::Mat camImage, std::vector<cv::Point2f>* scene_corners) {
	cv::Mat H;
	try {
		std::vector<cv::DMatch> good_matches;
		std::vector<cv::Point2f> obj;
		std::vector<cv::Point2f> scene;

		progImage = getImage();
		progImage = cv::imread("d:/work/custom.png", CV_LOAD_IMAGE_COLOR);
		cv::flip(progImage, progImage, 1);

		//-- Step 1: Detect the keypoints using SURF Detector

		detector.detect(progImage, keypoints_object);
		detector.detect(camImage, keypoints_scene);
		//-- Step 2: Calculate descriptors (feature vectors)
		extractor.compute(progImage, keypoints_object, descriptors_object);
		extractor.compute(camImage, keypoints_scene, descriptors_scene);
		//-- Step 3: Matching descriptor vectors using FLANN matcher
		if (cv::countNonZero(descriptors_object) < 1 || cv::countNonZero(descriptors_scene) < 1) {
			return cv::Mat();
		}
		matcher.match(descriptors_object, descriptors_scene, matches);
		//-- Step 4: find object
		//-- Quick calculation of max and min distances between keypoints
		for (int i = 0; i < descriptors_object.rows; i++)
		{
			double dist = matches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}
		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		double desired = max_dist - min_dist;
		desired = desired / 4.;
		desired = min_dist + desired;//in lower 3rd distance
		for (int i = 0; i < descriptors_object.rows; i++)
		{
			if (matches[i].distance < desired)
			{
				good_matches.push_back(matches[i]);
			}
		}
		if (good_matches.size() < 3) {
			return cv::Mat();
		}
		for (int i = 0; i < good_matches.size(); i++)
		{
			//-- Get the keypoints from the good matches
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}
		H = cv::findHomography(obj, scene, CV_RANSAC);
		//-- Get the corners from the image_1 ( the object to be "detected" )
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(progImage.cols, 0);
		obj_corners[2] = cvPoint(progImage.cols, progImage.rows);
		obj_corners[3] = cvPoint(0, progImage.rows);
		cv::Mat img_matches;
		drawMatches(progImage, keypoints_object, camImage, keypoints_scene,
			good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
			cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		perspectiveTransform(obj_corners, *scene_corners, H);

		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		cv::line(img_matches, (*scene_corners)[0] + cv::Point2f(progImage.cols, 0), (*scene_corners)[1] + cv::Point2f(progImage.cols, 0), cv::Scalar(0, 255, 0), 4);
		cv::line(img_matches, (*scene_corners)[1] + cv::Point2f(progImage.cols, 0), (*scene_corners)[2] + cv::Point2f(progImage.cols, 0), cv::Scalar(0, 255, 0), 4);
		cv::line(img_matches, (*scene_corners)[2] + cv::Point2f(progImage.cols, 0), (*scene_corners)[3] + cv::Point2f(progImage.cols, 0), cv::Scalar(0, 255, 0), 4);
		cv::line(img_matches, (*scene_corners)[3] + cv::Point2f(progImage.cols, 0), (*scene_corners)[0] + cv::Point2f(progImage.cols, 0), cv::Scalar(0, 255, 0), 4);

		cv::imshow("found", img_matches);
		return H;
	}
	catch (cv::Exception ex) {
		ex.formatMessage();
		return cv::Mat();
	}
}