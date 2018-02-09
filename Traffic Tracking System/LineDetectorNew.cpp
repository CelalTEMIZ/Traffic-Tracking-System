#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <pthread.h>
#include <raspicam/raspicam_cv.h>


using namespace std;
using namespace cv;

#define DEBUG 0

#define VIDEO_MODE 1
#define THRESHOLD_VIDEO 1

Point center;

bool flagVibration = false;

int flagSpeed = 0;
int  flagLine = 1;
int flagCarDetect = 0;
int flagSign = 0;

int flagStatus = 1;//Arac duruyorsa
int counterStatus[4] = {0,0,0,0};

int decreaseBottom = 0;
int decreaseTop = 0;

int decreaseLeft = 0;
int decreaseRight = 0;

int iLowH = 0;
int iHighH = 255;

int iLowS = 0;
int iHighS = 235;

int iLowV = 20;
int iHighV = 255;

int sens = 144;


void findAndDrawHoughLines(Mat &img, Mat& output, Mat &thresholdVideo, int thresholds[2], int frame,int startframe);

void FindSpeed(Mat imag1, Mat imag2);
double differenceBetPoints(vector<Point2f>& point1, vector<Point2f>& point2, vector<uchar> ishere, double rowNum, double colNum);

void draw_locations(Mat & img, const vector< Rect > & locations, const Scalar & color);



double distance_to_Line(cv::Point line_start, cv::Point line_end, cv::Point point){
	double normalLength = hypot(line_end.x - line_start.x, line_end.y - line_start.y);
	double distance = (double)((point.x - line_start.x) * (line_end.y - line_start.y) - (point.y - line_start.y) * (line_end.x - line_start.x)) / normalLength;
	return distance;
}

void *vibrate(void *arg){
  

    digitalWrite (0, HIGH) ; delay (500) ;
    digitalWrite (0,  LOW) ; delay (500) ;
  return NULL;
}

void GammaCorrection(Mat& src, Mat& dst, float fGamma){
	unsigned char lut[256];
	for (int i = 0; i < 256; i++){ 
	lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f); 
	}
	 
	dst = src.clone();
	 
	const int channels = dst.channels();
	 
	switch (channels)
	 
	{
	 
	case 1:
	 
	{
	 
	MatIterator_<uchar> it, end;
	 
	for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
	 
	*it = lut[(*it)];
	 
	break;
	 
	}
	 
	case 3:
	 
	{
	 
	MatIterator_<Vec3b> it, end;
	 
	for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++)
	 
	{
	 
	(*it)[0] = lut[((*it)[0])];
	 
	(*it)[1] = lut[((*it)[1])];
	 
	(*it)[2] = lut[((*it)[2])];
	 
	}
	 
	break;
	 
	}
	 
}
 
}



double getAngle(const Vec4i & line)
{
	return fabs(atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI);
}



bool pred(const Vec4i & a, const Vec4i &b)
{
	if (fabs(getAngle(a) - getAngle(b)) < 5)
	{
		return true;
	}
	return false;
}

struct partiti {
	bool operator()(const Vec4i & a, const Vec4i &b) {
		return pred(a, b);//sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)) < 15;
	}
};

double angle(Point2f &A, Point2f &B);
bool checkRectangle(std::vector<cv::Point> & cont);
bool checkTriangle(std::vector<cv::Point> & cont);


int main(void){

	String trackBarWinName = "Trackbars";
	int thresholds[2] = {20,180};
	namedWindow(trackBarWinName, CV_WINDOW_NORMAL);

pthread_t tid;
	wiringPiSetup () ;
  	pinMode (0, OUTPUT) ;
	digitalWrite (0,  LOW) ;



	cv::Mat inputImg, imgGRAY, dst,orginal;
	cv::Mat outputImg;
	cv::Size procSize;
	cv::VideoCapture video;
	std::vector<vector<cv::Point> > houghLines;

	CascadeClassifier car_cascade;
	car_cascade.load("cars3.xml");


	video.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	video.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	//Video file name
	char *videoName = "C:\\Users\\Kartakur\\Downloads\\AnkaraYolu.avi";//Path of video file
	//char *videoName = "C:\\Users\\Kartakur\\Downloads\\Yol01.avi";//Path of video file
	

	// Open video input
	if (VIDEO_MODE)
		video.open(0);
	else
	{
		video.open(videoName);
		//video.set(CAP_PROP_POS_FRAMES, 22000);
		//video.set(CAP_PROP_POS_FRAMES, 3000);
	}

	// Check video input
	int width = 0, height = 0;
	if (!video.isOpened())
	{
		if (DEBUG)
			printf("ERROR: can not open camera or video file\n");
		return -1;
	}
	else
	{
		// Show video information
		width = (int)video.get(CV_CAP_PROP_FRAME_WIDTH);
		height = (int)video.get(CV_CAP_PROP_FRAME_HEIGHT);
	}

	procSize = cv::Size(width, height);

	/*raspicam::RaspiCam_Cv Camera;
	//Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    
    	Camera.set(CV_CAP_PROP_FRAME_WIDTH,  800 );
   	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 600 );

	if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}


	int width = 0, height = 0;

	width = Camera.get(CV_CAP_PROP_FRAME_WIDTH);
	height = Camera.get(CV_CAP_PROP_FRAME_HEIGHT);*/


	int gamma=100;
float fgamma=1;


	namedWindow("LineOutput", WINDOW_NORMAL);
	namedWindow("Threshold Output", WINDOW_NORMAL);
	
	createTrackbar("BottomHeight", trackBarWinName, &decreaseBottom, height);
	createTrackbar("TopHeight", trackBarWinName, &decreaseTop, height);

	createTrackbar("LeftWidht", trackBarWinName, &decreaseLeft, width / 2);
	createTrackbar("RightWidth", trackBarWinName, &decreaseRight, width / 2);

	createTrackbar("Threshold1", trackBarWinName, &thresholds[0], 255);
	createTrackbar("Threshold2", trackBarWinName, &thresholds[1], 255);

	createTrackbar("sens", trackBarWinName, &sens, 255);
	createTrackbar("gamma", trackBarWinName, &gamma, 255);

	

	createTrackbar("Speed", trackBarWinName, &flagSpeed, 1);
	createTrackbar("Line", trackBarWinName, &flagLine, 1);
	createTrackbar("Car Detect", trackBarWinName, &flagCarDetect, 1);
	createTrackbar("Traffic Sign", trackBarWinName, &flagSign, 1);

	//createTrackbar("Threshold1", trackBarWinName, &thresholds[0], 255);
	//createTrackbar("Threshold2", trackBarWinName, &thresholds[1], 255);


	//Create trackbars in trackBarWinName window
	/*createTrackbar("LowH", trackBarWinName, &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", trackBarWinName, &iHighH, 179);

	createTrackbar("LowS", trackBarWinName, &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", trackBarWinName, &iHighS, 255);

	createTrackbar("LowV", trackBarWinName, &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", trackBarWinName, &iHighV, 255);*/

	

	center = Point(width / 2, height / 2);
	long int frame=0;
	long int startFrame = frame -1 ;

	Mat yeni, eski;
	Mat carFrame;

	
	

	

	vector<Rect> cars;


	video >> orginal;
	//resize(frame, frame, Size(640, 360));
	cvtColor(orginal, yeni, CV_BGR2GRAY);
	eski = yeni.clone();

	//Camera.grab();
    	//Camera.retrieve(orginal);

	//cvtColor(orginal, yeni, CV_BGR2GRAY);
	//eski = yeni.clone();

	Size kernalSize(4, 4);
	Mat element = getStructuringElement(MORPH_RECT, kernalSize, Point(1, 1));
	Mat mask, signGray, test;

	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contours2;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<Point> approx;

	while (1){

	

		//eski = yeni.clone();
		//video >> orginal;
		
		//video.set(CAP_PROP_POS_FRAMES, frame*2);
		video >> orginal;

		fgamma = ((float)gamma )/100;
		GammaCorrection(orginal,orginal,fgamma);
		//Camera.grab();
    		//Camera.retrieve(orginal);


		inputImg = orginal(Rect(decreaseLeft, decreaseTop, width - decreaseLeft - decreaseRight, height - decreaseTop - decreaseBottom));

		
		cvtColor(inputImg, yeni, CV_BGR2GRAY);
		
		//imshow("Car Detection", frame);



		if (flagSpeed){
			if (frame % 3 == 0){
				eski = yeni.clone();
			}


			if (frame % 3 == 2 && eski.size() == yeni.size()){
				FindSpeed(eski, yeni);
			}
		}

		
		if (flagSign){
			Canny(orginal, signGray, 200, 255, 3);
			morphologyEx(signGray, signGray, MORPH_CLOSE, element);

			cv::findContours(signGray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			for (unsigned i = 0; i < contours.size(); ++i)
			{
				contours2.push_back(approx);
				approxPolyDP(contours[i], contours2[i], 3, true);
			}

			//cv::drawContours(gray, contours2, -1, (0, 255, 0), 3);
			for (int i = 0; i < contours2.size(); ++i)
			{
				if (contours2[i].size() == 4 && checkRectangle(contours2[i]))
				{
					drawContours(orginal, contours2, i, (0, 255, 255), 3);
				}
			}

		}

		++frame;
		
		//imshow("yeni", yeni);
		//imshow("eski", eski);

		//imshow("Hýz Tespiti", frame);

		//Get current Frame		
		//video >> inputImg;
		
		

		//If we are in last frame
		if (inputImg.empty())
			break;


		//cvtColor(inputImg, imgGRAY, CV_BGR2GRAY);
		//inputImg.copyTo(outputImg);

		if (flagLine)
			findAndDrawHoughLines(inputImg, orginal, dst, thresholds, frame, startFrame);

		if (!flagVibration && startFrame != frame)
			startFrame = frame;



		if (flagCarDetect){
			cvtColor(orginal, carFrame, CV_BGR2GRAY);

			equalizeHist(carFrame, carFrame);
			car_cascade.detectMultiScale(carFrame, cars, 1.1, 2, CV_HAAR_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(25, 25));

			draw_locations(orginal, cars, Scalar(0, 0, 255));
		}

		if(flagVibration)
	  	{
     	 		pthread_create(&tid  ,NULL ,vibrate,NULL);
      			flagVibration=false;
   	  	}
		



		//cout << "flagStatus:" << flagVibration << "FrameGap: " << frame - startFrame << endl;

		imshow("LineOutput", orginal);
		if (THRESHOLD_VIDEO)
		{
			//circle(dst, center, 2, Scalar(255, 255, 255), 2);
			imshow("Threshold Output", dst);
		}
			

		if(cv::waitKey(8)==27){
			digitalWrite (0,  LOW) ;
			return 0;
		}

	}


}


double differenceBetPoints(vector<Point2f>& point1, vector<Point2f>& point2, vector<uchar> ishere, double rowNum, double colNum){
	double average = 0.0, distance = 0.0;
	Point2f diff;
	int gap=4;

	vector<Point2f> oldPoints, newPoints;

	for (int i = 0; i < point1.size(); i++){
		if (ishere.at(i) == 0){
			diff.x = rowNum / 100; diff.y = colNum / 100;
		}
		else
			diff = point2.at(i) - point1.at(i);
		distance += sqrt(fabs(diff.x*diff.x + diff.y*diff.y));
	}

	average = distance / point1.size();
	average *= 100;

	if (average < 200){
		counterStatus[0]++;
		if (counterStatus[0] == gap){
			cout << " \n\nDURUYOR";
			flagStatus = 0;
			counterStatus[0] = 0;
		}	
	}
	else if (average < 700){
		counterStatus[1]++;
		if (counterStatus[1] == gap){
			cout << "\n\nYAVAS GIDIYOR";
			flagStatus = 1;
			counterStatus[1] = 0;

		}
		
	}
	else if (average < 1500){
		counterStatus[2]++;
		if (counterStatus[2] == gap){
			cout << "\n\nHIZLI GIDIYOR";
			flagStatus = 2;
			counterStatus[2] = 0;
		}
		
	}
	else{
		counterStatus[3]++;
		if (counterStatus[3] == gap){
			cout << "\n\nCOK HIZLI GIDIYOR";
			flagStatus = 3;
			counterStatus[3] = 0;
		}
		
	}
	if (DEBUG)
		cout << "     Distance:  " << average;

	/*	*/
	return average;
}


void FindSpeed(Mat imag1, Mat imag2){

	int row = 10, col = 10;
	double distance = 0;
	int x, y;
	vector<float> hata; //hesaplama hatasý
	vector<uchar> tp; /*	*/
	vector<Point2f> oldPoints, newPoints;


	//cout << "\n" << imag1.rows << "  "<<imag1.cols;

	row++; col++;
	for (x = 1; x <row; x++){
		for (y = 1; y < col; y++){
			oldPoints.push_back(Point2f(y*imag1.cols / col, x*imag1.rows / row));
		}
	}

	//cout << "\nNoktalar taraniyor.. ["<<point.size()<<"]";
	//cout << "\n" << oldPoints.size()<<"***************************";

	calcOpticalFlowPyrLK(imag1, imag2, oldPoints, newPoints, tp, hata);

	/*for (int i = 0; i < oldPoints.size(); i++){
		//cout << " \n" << ". POÝNT : " << oldPoints.at(i).x << " " << oldPoints.at(i).y;
		line(frame, oldPoints.at(i), oldPoints.at(i), Scalar(1, 1, 255), 5);
		line(frame, oldPoints.at(i), newPoints.at(i), Scalar(1, 1, 255), 1);
	}*/
	/*	*/

	differenceBetPoints(oldPoints, newPoints, tp, imag1.rows, imag1.cols);

	oldPoints.clear();
	newPoints.clear();


}


void findAndDrawHoughLines(Mat &img, Mat& output, Mat &thresholdVideo,int thresholds[2], int frame, int startframe){
	//Image-video Variables

	vector<vector<cv::Point> > lineSegs;
	cv::Mat imgGRAY;
	//cv::Mat dst;
	std::vector<cv::Vec4i> lines;
	vector<cv::Point> lSeg;
	cv::Point p1, p2;
	Mat hsv;
	

	cv::cvtColor(img, hsv, COLOR_BGR2HSV);
	int sensitivy = 70;
	cv::inRange(hsv, Scalar(0, 0, 255-sens), Scalar(255, sens, 255), imgGRAY);

	//Video Processing - Editable Area
	//cv::cvtColor(imgGRAY, imgGRAY, CV_HSV2BGR);
	//cv::cvtColor(imgGRAY, imgGRAY, CV_BGR2GRAY);
	//threshold(imgGRAY, imgGRAY,)
	
	cv::Canny(imgGRAY, thresholdVideo, thresholds[0], thresholds[1], 3, true);
	cv::HoughLinesP(thresholdVideo, lines, 1, CV_PI / 180, 50, 50, 10);

	//Threshold Video
	//if (THRESHOLD_VIDEO)	thresholdVideo = dst;


	double slope,degree;
	slope = 0;
	double result;
	int counter=0;
	

	int width = thresholdVideo.size().width;
	int height = thresholdVideo.size().height;

	center = Point(width / 2, height / 2);

	circle(thresholdVideo, center, 2, Scalar(255, 255, 255), 2);

	for (size_t i = 0; i<lines.size(); i++)
	{
		lSeg.clear();
		p1.x = lines[i][0];
		p1.y = lines[i][1];

		p2.x = lines[i][2];
		p2.y = lines[i][3];


		if ((p2.x - p1.x) !=0)
			slope = fabs((double)(p2.y - p1.y) / (double)(p2.x - p1.x));


		degree = atan(slope) * 180 / CV_PI;

		if (DEBUG){
			//cout << "Added : " << p1 << " : " << p2 << endl;
			cout << degree<<" - "<<slope << endl;

		}
		
		
		
		

		if ((degree >10 && degree <80 )){

			p1.y += decreaseTop;
			p2.y += decreaseTop;

			p1.x += decreaseLeft;
			p2.x += decreaseLeft;


			lSeg.push_back(p1);
			lSeg.push_back(p2);
			lineSegs.push_back(lSeg);
		}

		/*if (slope > 0.15 && slope < 0.15){
			p1.y += decreaseTop;
			p2.y += decreaseTop;

			p1.x += decreaseLeft;
			p2.x += decreaseLeft;


			lSeg.push_back(p1);
			lSeg.push_back(p2);
			lineSegs.push_back(lSeg);
		}
		*/

	}


	Scalar rightColor = CV_RGB(232, 12, 61);
	Scalar leftColor = CV_RGB(0, 255, 255);
	Scalar midColor = CV_RGB(255, 255, 0);
	center.x += decreaseLeft;
	center.y += decreaseTop;

	

	for (size_t i = 0; i<lineSegs.size(); i++){
		if (lineSegs[i][0].x < width / 2 && lineSegs[i][1].x < width / 2){
			line(output, lineSegs[i][0], lineSegs[i][1], rightColor, 5);
		}
		else if (lineSegs[i][0].x >= width / 2 && lineSegs[i][1].x >= width / 2){
			line(output, lineSegs[i][0], lineSegs[i][1], leftColor, 5);
		}
		else{
			line(output, lineSegs[i][0], lineSegs[i][1], midColor, 2);
		}

		
		result = distance_to_Line(lineSegs[i][0], lineSegs[i][1], center);
		//line(output, lineSegs[i][0], center, CV_RGB(255, 0, 0), 2);
		circle(output, center, 2, Scalar(255, 255, 255), 2);
		//cout <<"--"<< result << endl;

		if (fabs(result) < width / 30){
			++counter;
		}

		if (counter > 1 && !flagVibration){
			cout << "Serit degisti" << i << endl;
			flagVibration = true;
		}

	}





}


void draw_locations(Mat & img, const vector<Rect> & locations, const Scalar & color)
{

	int thickness = 1;

	if (!locations.empty())
	{
		vector<Rect>::const_iterator loc = locations.begin();
		vector<Rect>::const_iterator end = locations.end();

		for (; loc != end; ++loc)
		{
			rectangle(img, *loc, color, thickness);
		}
	}
}

bool checkRectangle(std::vector<cv::Point> & cont){
	double val;
	int h = 0;
	int v = 0;
	int vh = 0;/*1=v,2=h,0=none*/
	for (int i = 0; i < 3; ++i)
	{
		val = (cont[i + 1].y*1.0 - cont[i].y*1.0) / (cont[i + 1].x*1.0 - cont[i].x*1.0); // calculate slope between the two points
		val = fabs(val);
		if (val < 0.03 && vh != 2)
		{
			++h;
			vh = 2;
		}
		else if ((1.03 < val  && 0.97 < val && vh != 1))
		{
			++v;
			vh = 1;
		}
		else
		{
			return false;
		}

	}
	val = (cont[0].y*1.0 - cont[3].y*1.0) / (cont[0].x*1.0 - cont[3].x*1.0); // calculate slope between the two points
	val = fabs(val);
	if (val < 0.03 && vh != 2)
	{
		++h;
		vh = 2;
	}
	else if ((1.03 < val  && 0.97 < val && vh != 1))
	{
		++v;
		vh = 1;
	}
	else
	{
		return false;
	}
	if (h == 2 && v == 2)
	{
		return true;
	}
	return false;
}

bool checkTriangle(std::vector<cv::Point> & cont){
	double val;
	int h = 0;
	int v = 0;
	int vh = 0;/*1=v,2=h,0=none*/
	for (int i = 0; i < 3; ++i)
	{
		val = (cont[i + 1].y*1.0 - cont[i].y*1.0) / (cont[i + 1].x*1.0 - cont[i].x*1.0); // calculate slope between the two points
		val = fabs(val);
		if (val < 0.03 && vh != 2)
		{
			++h;
			vh = 2;
		}
		else if ((1.03 < val  && 0.97 < val && vh != 1))
		{
			++v;
			vh = 1;
		}
		else
		{
			return false;
		}

	}
	val = (cont[0].y*1.0 - cont[3].y*1.0) / (cont[0].x*1.0 - cont[3].x*1.0); // calculate slope between the two points
	val = fabs(val);
	if (val < 0.03 && vh != 2)
	{
		++h;
		vh = 2;
	}
	else if ((1.03 < val  && 0.97 < val && vh != 1))
	{
		++v;
		vh = 1;
	}
	else
	{
		return false;
	}
	if (h == 2 && v == 2)
	{
		return true;
	}
	return false;
}

double angle(Point2f & A, Point2f& B) {
	double val = (B.y - A.y) / (B.x - A.x); // calculate slope between the two points
	val = val - pow(val, 3) / 3 + pow(val, 5) / 5; // find arc tan of the slope using taylor series approximation
	val = ((int)(val * 180 / 3.14)) % 360; // Convert the angle in radians to degrees
	if (B.x < A.x) val += 180;
	if (val < 0) val = 360 + val;
	return val;
}


