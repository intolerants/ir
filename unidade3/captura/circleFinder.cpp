/*
    UFRN 2016.1 - INTRODUCAO A ROBOTICA
            HANOCH E TAYNARA

  CAPTURA DE IMAGEM E DETECTCAO DE CIRCULOS
*/

// Ver em: http://hxr99.blogspot.com.br/2011/12/opencv-examples-camera-capture.html
// -------------------------------------------------------------------------------
//  OpenCV C/C++ Examples (Camera Capture)
// There are also python implementation for camera capture and showing image.

// Compile and run: g++ circleFinder.cpp `pkg-config opencv --cflags` `pkg-config opencv --libs` -o circleFinder && ./circleFinder 


#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "findpath.h"


// #define DEBUG 1
#define BLUE 223
#define YELLOW 238
#define BLACK 21
#define NUM_OF_COLORS 3
#define OFFSET 350
#define OFFSETY 350
#define CONTRAST 0.2
#define BRIGHTNESS 20
#define aBLUR 2
#define WIDTH 1400
#define RADIUS 35
#define NUM_OF_ELEMENTS 7

using namespace std;
using namespace cv;

void takePicture(void);
void initFindCircles(void);
int findCircles(void);
char checkColor(Point center);
void openFile(void);


int COLORS[NUM_OF_COLORS] = {YELLOW, BLUE, BLACK};
char ANSWERS[NUM_OF_COLORS] = {'b', 'e', 'w'}; //end, begin, wall
// int COLORS[NUM_OF_COLORS] = {BLUE, YELLOW, BLACK};
// char ANSWERS[NUM_OF_COLORS] = {'e', 'b', 'w'}; //end, begin, wall
float LIMITS[NUM_OF_COLORS - 1];
Mat src, srcHsv, image, workMap, detectionMap;
int windowCont = -1;
double brightness = -160;
double contrast = 2;
// int ablur = -1;
int ablur = 11;
Point pTemp;
Point element[NUM_OF_ELEMENTS];
char elementKind[NUM_OF_ELEMENTS];
Rect box;
int world_map2[330*400];

bool foundStart, foundEnd;

Point windowPos(void) {
	int pos = ++windowCont * OFFSET;
	return Point((pos % WIDTH), (((pos / WIDTH)) * OFFSETY));
}

void showWindow(Mat m, string s)
{
	namedWindow(s, CV_WINDOW_FREERATIO);
	imshow(s, m);
	resizeWindow(s, OFFSET, OFFSET);
	pTemp = windowPos();
#ifdef DEBUG
	cout << "----" << s << "(" << pTemp.x << " ," << pTemp.y << ")" << endl;
#endif
	moveWindow(s, pTemp.x, pTemp.y);
}

void drawMap(void) {
	// src.copyTo(workMap);
	// workMap.convertTo(workMap, -1, 1, 1000);
	rectangle(workMap, Rect(0,0,330,400), Scalar(255, 255, 254), -1);
	// rectangle(workMap, Rect(2,60,325,227), Scalar(0, 0, 0), 2);
	for (int i = 0; i < NUM_OF_ELEMENTS; i++)
	{
		if (elementKind[i] == 'w') {
			// circle( workMap, element[i], RADIUS, Scalar(0, 0, 255), 3, 8, 0 ); // circle outline
			circle( workMap, element[i], RADIUS, Scalar(0, 0, 255), -1, -1, 0 ); // circle outline
		}
		else {
			Vec3b centerColor = Vec3b(0, 0, 0); // error = black
			if (elementKind[i] == 'e')
				centerColor = Vec3b(255, 0, 1); //BGR end = blue
			else if (elementKind[i] == 'b')
				centerColor = Vec3b(0, 255, 253); //BGR begin = yellow

			// circle( workMap, element[i], 1, centerColor, 0, 8, 0 ); // circle center
			workMap.at<Vec3b>(element[i]) = centerColor;
			// srcHsv.at<Vec3b>(center + Point(size  / 4  - i,  i)) = Vec3b(255, 255, 255);


		}
	}
#ifdef DEBUG
	showWindow(workMap, "Espaço de Configuração");
#endif
}

int convertMatrix(int result[][2]){
	int size, start[2], end[2];

	// cout << "Matriz" << endl;
	for (int i = 0; i < 330; i++){
		for (int j = 0; j < 400; j++){
			// cout << world_map2[i*330 + j] << " ";
			if (workMap.at<Vec3b>(Point(i, j)).val[2] == 1){
				workMap.at<Vec3b>(Point(i, j)) = Vec3b(100,100,0);
				end[0] = i;
				end[1] = j;

#ifdef DEBUG
				// cout << "endColor:" << (int)workMap.at<Vec3b>(Point(i, j)).val[2] << endl;
				cout << "end: " << end[0] << " " << end[1] << endl;
#endif

			}
			else if (workMap.at<Vec3b>(Point(i, j)).val[2] == 253){
				workMap.at<Vec3b>(Point(i, j)) = Vec3b(100,100,0);
				start[0] = i;
				start[1] = j;

#ifdef DEBUG
				// cout << "startColor:" << (int)workMap.at<Vec3b>(Point(i, j)).val[2] << endl;
				cout << "start: " << start[0] << " " << start[1] << endl;
#endif
				
			}
			world_map2[j*330 + i] = (int)workMap.at<Vec3b>(Point(i, j)).val[2] == 255 ? 9:1;
		}
		//cout << endl;
	}

	size = findPath(world_map2, result, start, end);

	Vec3b pathColor = Vec3b(0,0,0);
	// cout << "Result" << endl;
    for (int i = 0; i < size; i++){
        // cout << result[i][0] << " " << result[i][1] << endl;
		workMap.at<Vec3b>(Point(result[i][0],result[i][1])) = pathColor;
		// cout << endl;
	}
	showWindow(workMap, "path");
	return size;
}

int main(int argc, const char** argv)
// void run(void)
{	
	int size, result[10000][2];
	initFindCircles();
	takePicture();
	// findCircles();
	drawMap();
	
	size = convertMatrix(result);
	// for (int i = 0; i < size; i++){
	// 	for (int j = 0; j < 2; j++)
	// 		cout << result[i][j] << " ";
	// 	cout << endl;
	// }
#ifdef DEBUG
	// showWindow(src, "Source");
	cout << "press ESC to exit" << endl;
	while((cv::waitKey() & 0xEFFFFF) != 27); //27 is the keycode for ESC
#endif
	destroyAllWindows();

	return 0;
}

void takePicture() {
  CvCapture* capture = 0;
  Mat frame, frameCopy;

  capture = cvCaptureFromCAM( -1 ); //0=default, -1=any camera, 1..99=your camera

#ifdef DEBUG
  if (!capture)
      cout << "No camera detected" << endl;
#endif

  // cvNamedWindow("result", CV_WINDOW_AUTOSIZE);

  if (capture)
  {

#ifdef DEBUG
      cout << "In capture ..." << endl;
#endif

    for (;;)
    {
      IplImage* iplImg = cvQueryFrame( capture );
      frame = iplImg;

      if ( frame.empty() )
        break;
      if ( iplImg->origin == IPL_ORIGIN_TL )
        frame.copyTo( frameCopy );
      else
        flip( frame, frameCopy, 0 );

      image = cvarrToMat(iplImg);
      // cvShowImage( "result", iplImg );
      // cvSaveImage("img.png", iplImg);
      windowCont = -1;
      int numOfCircles = findCircles();

#ifdef DEBUG
      cout << "Found " << numOfCircles << " circles";
#endif

      if ( waitKey( 10 ) >= 0 || (numOfCircles == 7 && foundStart && foundEnd))
        // if (DEBUG)
          break;
      cout << " =========== TRYING AGAIN ============\n\n";
    }
    /*IplImage* iplImg = cvQueryFrame(capture);
    cvShowImage("result", iplImg);
    cvSaveImage("img.png", iplImg);
    waitKey(0);*/
  }


  cvReleaseCapture(&capture);
  // cvDestroyWindow("result");
}

void initFindCircles(void) {
	for (int i = 0; i < NUM_OF_COLORS - 1; ++i)
	{
		LIMITS[i] = (COLORS[i] - COLORS[i + 1]) / 2.0 + COLORS[i + 1];
#ifdef DEBUG
		cout << "LIMITS[" << i << "]:" << LIMITS[i] << endl;
#endif
	}
}

int findCircles() {

	// while (1) {
	// initFindCircles();
	Mat gray;
	Mat canny;

	/*
	  Transform it into the C++ cv::Mat format
	*/
	// image = imread("1eComFundo.jpg", 1);

	/*
	  Cria retangulo para filtragem da area de trabalho
	*/
	// Rect box = Rect(300, 57, 330, 400); //Vostro FOV workspace
	box = Rect(300, 50, 330, 400); //LAB FOV workspace
	// Rect box = Rect(320, 157, 277, 200); //Vostro FOV scanner
	// Rect box = Rect(360, 170, 210, 180); //Vostro FOV imagem
	// Rect box = Rect(360, 170, 200, 170);
	// Rect box = Rect(560, 260, 300, 260);

	/*
	  Setup a rectangle to define your region of interest
	*/
	Mat src(image, box);
	detectionMap = src.clone();
	// workMap = src.clone();
	src.copyTo(workMap);


	//cvtColor(src, src, CV_8U);

	/*
	  Carrega a matriz para o reconhecimento de cores
	*/
	srcHsv = src.clone();
	medianBlur(srcHsv, srcHsv, 11);
	srcHsv.convertTo(srcHsv, -1, 1.2, -70);
	cvtColor(src, srcHsv, COLOR_BGR2HSV);
	medianBlur(srcHsv, srcHsv, 11);


#ifdef DEBUG
	showWindow(src, "Source");
	showWindow(srcHsv, "srcHsv0");
#endif
	/*
	  Carrega a matriz para o reconhecimento dos circulos
	*/
	cvtColor(src, gray, CV_RGB2GRAY);
	// cvtColor( src, gray, CV_BGR2GRAY );

	/*
	  Filtragem - Escurecer o amarelo
	*/
	// gray.convertTo(gray, -1, 1.3, -200);
	Mat graySource;
	cvtColor(src, graySource, CV_RGB2GRAY);
	graySource.convertTo(graySource, -1, 2.6, -300);
	medianBlur(graySource, graySource, 11);


	// blur(gray, gray, Size( 5, 5 ), Point(-11, -11));
	// medianBlur(gray, gray, 7);
	medianBlur(gray, gray, ablur);
	gray.convertTo(gray, -1, contrast, brightness);
	// contrast:2.6 brightness:-260 blur:11

#ifdef DEBUG
	showWindow(gray, "gray");
	showWindow(graySource, "graySource");
#endif
	/*
	  Filtragem - Escurecer o filtragem pre-canny contornos
	*/

	// bitwise_or(graySource, gray, gray);


#ifdef DEBUG
	showWindow(gray, "blur");
#endif

	/*
	  Filtragem - Cria matriz de contornos
	*/
	Canny(gray, canny, 10, 140, 3, true);
#ifdef DEBUG
	showWindow(canny, "canny1");
#endif

	/*
	  Filtragem - Aumenta bordas
	*/
	blur(canny, canny, Size( 3, 3 ), Point(-1, -1));
#ifdef DEBUG
	showWindow(canny, "canny3");
#endif

	/*
	  Filtragem - Suaviza imperfeicoes
	*/
	medianBlur(canny, canny, 5);
#ifdef DEBUG
	showWindow(canny, "canny3Blur");
#endif
	// moveWindow("canny3", 720+220, 0);



	/*
	  Apply the Hough Transform to find the circles
	*/
	vector<Vec3f> circles;
	// HoughCircles(InputArray, OutputArray, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
	// HoughCircles(   canny,      circles,      CV_HOUGH_GRADIENT, 0.5,         0.1,                  100,               100,            0,                  0);
	HoughCircles(canny, circles, CV_HOUGH_GRADIENT, 1, 40, 40, 30, 5, 50);
	// HoughCircles(canny, circles, CV_HOUGH_GRADIENT, 0.5, 30, 200, 50, 0, 0);


	// openFile();
	system("rm pos.txt");
	system("cat /dev/null > pos.txt");

	FILE *file = fopen("pos.txt", "a");
	if (file == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}

	/*
	  Draw the circles detected
	*/
	detectionMap.convertTo(detectionMap, -1, 1, 250);

	char idCircle;
	foundEnd = false;
	foundStart = false;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		idCircle = checkColor(center); //find color
#ifdef DEBUG
		cout << "center : " << center << "\tradius : " << radius << "\thue: " << idCircle << "\tcolor: " << (int)srcHsv.at<Vec3b>(center).val[0] << " " << (int)srcHsv.at<Vec3b>(center).val[1] << " " << (int)srcHsv.at<Vec3b>(center).val[2] << endl;
#endif
		fprintf(file, "%d %d %c\n", center.x, center.y, idCircle);
		// circle( detectionMap, center, 3, Scalar(0, 255, 0), -1, 8, 0 ); // circle center
		// circle( detectionMap, center, radius, Scalar(0, 0, 255), -1, -1, 0 ); // circle outline
		if (idCircle == 'w') {
			// circle( workMap, element[i], RADIUS, Scalar(0, 0, 255), 3, 8, 0 ); // circle outline
			circle( detectionMap, center, radius, Scalar(0, 0, 255), -1, -1, 0 ); // circle outline
			// circle( workMap, element[i], RADIUS, Scalar(0, 0, 255), -1, -1, 0 ); // circle outline
		} else if (idCircle == 'e') {
			foundEnd = true;

			circle( detectionMap, center, 3, Scalar(255, 0, 0), -1, -1, 0 ); // circle outline
		} else if (idCircle == 'b') {
			foundStart = true;

			circle( detectionMap, center, 14, Scalar(0, 255, 255), -1, -1, 0 ); // circle outline

		}
		element[i] = center;
		elementKind[i] = idCircle;
	}

	fclose(file);

	// Show your results
#ifdef DEBUG
	showWindow(detectionMap, "Espaço de trabalho");
#endif
	imwrite( "./circulos.png", detectionMap );

	// namedWindow("color"); imshow("color", srcHsv);
	// moveWindow("color", 0, 250);

#ifdef CONTRAST_CALIBRATION
	switch ((char)waitKey(0)) {
	case 'd':
		contrast += CONTRAST;
		break;
	case 'c':
		contrast -= CONTRAST;
		break;
	case 'g':
		brightness += BRIGHTNESS;
		break;
	case 'b':
		brightness -= BRIGHTNESS;
		break;
	case 'r':
		ablur += aBLUR;
		break;
	case 'f':
		ablur -= aBLUR;
		break;
	case 'x':
		return circles.size();
		break;
	}
	// windowCont = -1;
	// destroyAllWindows();
#endif

#ifdef DEBUG
	cout << "contrast:" << contrast << "\tbrightness:" << brightness << "\tblur:" << ablur << endl;
#endif

#ifdef CONTRAST_CALIBRATION
}
#endif

return circles.size();
}


char checkColor(Point center) {
	int size = 100;
	int sample[3], hue, saturation;
	char hueDebug;
	for (int j = 0; j < 3; ++j)
	{
		sample[j] = 0;
		for (int i = 0; i < size / 4; ++i)
		{
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(size / 4 - i, i)).val[j];
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(size / 4  - i, -i)).val[j];
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(-size / 4  + i, i)).val[j];
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(-size / 4  + i, -i)).val[j];
		}
	}

#ifdef DEBUG
	for (int i = 0; i < size / 4; ++i)
	{
		srcHsv.at<Vec3b>(center + Point(size  / 4  - i,  i)) = Vec3b(255, 255, 255);
		srcHsv.at<Vec3b>(center + Point(size  / 4  - i, -i)) = Vec3b(255, 255, 255);
		srcHsv.at<Vec3b>(center + Point(-size / 4  + i,  i)) = Vec3b(255, 255, 255);
		srcHsv.at<Vec3b>(center + Point(-size / 4  + i, -i)) = Vec3b(255, 255, 255);
	}
	// showWindow(srcHsv, "checkColor");
	cout << "HSV:" << sample[0] / size << " " << sample[1] / size << " " << sample[2] / size << " - ";
#endif
	saturation = sample[1] / size;
	for (int i = 0; i < NUM_OF_COLORS - 1; ++i)
	{
		if (saturation > LIMITS[i])
			return ANSWERS[i];
	}
	return ANSWERS[NUM_OF_COLORS - 1];
	// return ANSWERS[NUM_OF_COLORS - 1];
}

void openFile(void) {
	system("rm pos.txt");
	system("cat /dev/null > pos.txt");

	FILE *file = fopen("pos.txt", "a");
	if (file == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}

	fclose(file);
}