/*
Spencer Waguespack
Mohawk Rug flipping
1/28/2020

description
give image return center and orientaion
the steps are
1. blurring
2. thresholding operation
3. run a opening image to remove bright spots
4. run canny edge detection
5. find center using contours


line class is for helping to find the corners of the rugs
it stores lines as slope and intercept

//
//  Hello World server in C++
//  Binds REP socket to tcp://*:5555
//  Expects "Hello" from client, replies with "World"
//

*/

//std lib modules
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

//opencv modules
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <zmq.hpp>
#include <string>
#include <sstream>
#include <math.h>

#define PI 3.14159265

using namespace cv;

RNG rng(6969);

Mat src, blurred,dst_gray, closed, edges, detectedEdges;

int bler = 7;//set inital blur value and create blur global variable
int thresh = 106; //set inital threshold for thresholding value and create thresh global variable
int closeKern = 13; //set kernal and adjust kernal
int edgeThresh = 2;//set edge threshold
int edgeKern = 3;//sets edge kernal for edge detection
int edgeRatio = 3;//sets edge ratio between low and high thresh
int cornthresh = 1;
int apertureSize = 9;
int maxCorners = 3;
int maxTrackbar = 100;

std::vector<Point2f> corners;
double qualityLevel = 0.01;
double minDistance = 100;
int blockSize = 3, gradientSize = 3;
bool useHarrisDetector = false;
double k = 0.01;

int const maxEdgeThresh = 100;//maximum edge thershold
int const maxBlur = 55; //set max blur value
int const maxThresh = 255; //set max threshold value
int const maxKern = 21;//max morph kernal
int const maxcornthresh = 255;

std::string findrug(){

	medianBlur(src, blurred, bler);

	cvtColor(blurred, dst_gray, COLOR_BGR2GRAY);
	threshold(dst_gray, dst_gray, thresh, maxThresh, THRESH_BINARY);

	// Since MORPH_X : 2,3,4,5 and 6
	int operation = MORPH_OPEN;
	Mat element = getStructuringElement(0, Size(2 * closeKern + 1, 2 * closeKern + 1), Point(closeKern, closeKern));
	morphologyEx(dst_gray, closed, operation, element);

	Canny(closed, detectedEdges, edgeThresh, edgeThresh*edgeRatio, edgeKern);

	std::vector < std::vector <Point>> contours;
	std::vector<Vec4i> hierarchy;

	findContours(detectedEdges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	std::vector<std::vector<Point>> hull(contours.size());

	for (int i = 0; i < contours.size(); i++) {
		convexHull(contours[i], hull[i]);
	}

	std::vector<Moments> mu(hull.size());
	for (int i = 0; i < hull.size(); i++) {
		mu[i] = moments(hull[i]);
	}

	std::vector<Point2f> mc(hull.size());
	for (int i = 0; i < hull.size(); i++)
	{
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	/// Draw contours
	Mat drawing = Mat::zeros(detectedEdges.size(), CV_8UC3);
	std::cout << "centers\n";
	float cent[2];
	
	for (int i = 0; i < hull.size(); i++)
	{
		Scalar color = Scalar(255,255,255);
		drawContours(drawing, hull, i, color, FILLED, 8, hierarchy, 0, Point());
		circle(drawing, mc[i], 4, color, -1, 8, 0);
		std::cout << mc[i] << std::endl;
		
	}

	imshow("detected", detectedEdges);

	Mat corns = Mat::zeros(src.size(), CV_32FC1);
	maxCorners = MAX(maxCorners, 1);
	
	goodFeaturesToTrack(detectedEdges,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		gradientSize,
		useHarrisDetector,
		k);	
	Mat corns_norm, corns_norm_scaled;

	std::cout << "** Number of corners detected: " << corners.size() <<std::endl;
	int radius = 8;
	for (size_t i = 0; i < corners.size(); i++)
	{
		circle(drawing, corners[i], radius, Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), FILLED);
	}

	double nline,lline = 10000000;

	int x1,x2;
	int y1,y2;
	Point p1, p2;

	int lthickness = 4;
	for (size_t i = 0; i < corners.size(); i++)
	{
		x1 = corners[i].x;
		y1 = corners[i].y;
		if ( i < corners.size() - 1) {
			x2 = corners[i+1].x;
			y2 = corners[i+1].y;
		}
		else
		{
			x2 = corners[0].x;
			y2 = corners[0].y;
		}

		nline = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) ^ 2);
		
		if (nline < lline) {
			lline = nline;
			p1.x = x1;
			p1.y = y1;
			p2.x = x2;
			p2.y = y2;
		}

	}
	
	std::cout << lline << std::endl;
	line(drawing, p1, p2, Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), lthickness, LINE_AA);

	Point midpoint;
	
	midpoint.x = (p1.x + p2.x)/2;
	midpoint.y = (p1.y + p2.y)/2;

	std::cout << midpoint << std::endl;

	circle(drawing, midpoint, radius, Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), FILLED);
	line(drawing, midpoint, mc[0], Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), lthickness, LINE_AA);
	//namedWindow(corners_window);
	float slope = (midpoint.y-mc[0].y)/(midpoint.x-mc[0].x);
	float theta = atan(slope) * 180/PI;
	
	cent[0] = mc[0].x;
	cent[1] = mc[0].y;
	cent[2] = theta;
	
	imshow("output", drawing);
	waitKey();//wait for key to close window
	std::ostringstream oss;
	oss << "[(";
	
	
	for (int i = 0; i < 3; i++) {
      		oss << std::setprecision(10) << std::fixed << cent[0] <<", "<<cent[1]<<", "<<cent[2]<< " ";  // Use fixed in to avoid scientific notation, which is not handled on receiving end//(sw) litterally the most ghetto way to do this

      		//std::cout << matd_get(pose.t, i, 0) << std::endl;
    	}
    	oss << ")]";
    	//std::cout << std::endl << std::endl;
    	//std::cout << "pose\n";
  	
	return oss.str();
}

int main(int argc, char ** argv) {

	
	CommandLineParser parser(argc, argv, "{@input | IMG_3692.JPG | input image}");
	src = imread(parser.get<String>("@input"), IMREAD_REDUCED_COLOR_8);

	
	if (src.empty()) { //check for image
		std::cout << "Could not open or find the image!\n" << std::endl;
		std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
		return EXIT_FAILURE;
	}

  // Prepare our context and socket
  zmq::context_t context(1);
  zmq::socket_t socket(context, ZMQ_REP);
  socket.bind("tcp://127.0.0.1:43001");
	
	while (true){
	   // Wait for next request from client
    std::cout << "Server awaiting message " << std::endl;
    zmq::message_t request;
    std::string request_str;
    socket.recv(&request);
    request_str = std::string(static_cast<char*>(request.data()), request.size());
    std::cout << "request_str = " << request_str << std::endl;
    // Deal with request and create response string
    std::string response_str = "{";

    if (0 == request_str.compare("Request peg poses")){
	

	response_str += findrug() + "}";}
/*      
    else if (0 == request_str.compare("Request spool circles"))
      response_str += GetCircles(m_Cam) + "}";
      
    else if (0 == request_str.compare("Request image"))
      response_str = GetImagePath(m_Cam);
*/
    else // Did not recieve the right message      
      response_str = "{\"Error\"}";

    std::cout << response_str << std::endl;

    //  Send reply back to client
    zmq::message_t reply(response_str.size());
    memcpy(reply.data(), response_str.data(), response_str.size());
    socket.send(reply);
    
  }  // End while loop
	//findrug(0,0);


	return 0;
}
