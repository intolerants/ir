/*
    UFRN 2016.1 - INTRODUCAO A ROBOTICA
            HANOCH E TAYNARA

  CAPTURA DE IMAGEM E DETECTCAO DE CIRCULOS
*/

// Ver em: http://hxr99.blogspot.com.br/2011/12/opencv-examples-camera-capture.html
// -------------------------------------------------------------------------------
//  OpenCV C/C++ Examples (Camera Capture)
// There are also python implementation for camera capture and showing image.
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


#define DEBUG 1
#define RED 175
#define PINK 154
#define BLUE 108
#define CIAN 101
#define GREEN 75
#define YELLOW 28
#define BALCK_SATURATION 167
#define NUM_OF_COLORS 6
#define OFFSET 200

using namespace std;
using namespace cv;

void takePicture(void);
void initFindCircles(void);
int findCircles(void);
char checkColor(Point center);
void openFile(void);

int COLORS[NUM_OF_COLORS] = {RED, PINK, BLUE, CIAN, GREEN, YELLOW};
char ANSWERS[NUM_OF_COLORS] = {'r', 'p', 'b', 'c', 'g', 'y'};
float LIMITS[NUM_OF_COLORS - 1];
Mat src, srcHsv, image;
int windowCont = 0;


int main(int argc, const char** argv)
{
	initFindCircles();
	takePicture();
	return 0;
}

void takePicture() {
	CvCapture* capture = 0;
	Mat frame, frameCopy;

	capture = cvCaptureFromCAM( -1 ); //0=default, -1=any camera, 1..99=your camera

	if (!capture)
	{
		if (DEBUG)
			cout << "No camera detected" << endl;
	}

	// cvNamedWindow("result", CV_WINDOW_AUTOSIZE);

	if (capture)
	{
		if (DEBUG)
			cout << "In capture ..." << endl;
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
			if (DEBUG)
			{
				cvShowImage( "result", iplImg );
				cvSaveImage("img.png", iplImg);
			}

			int numOfCircles = findCircles();
			if ( waitKey( 10 ) >= 0)
		}
		waitKey(0);
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("result");
}

void initFindCircles(void) {
	for (int i = 0; i < NUM_OF_COLORS - 1; ++i)
	{
		LIMITS[i] = (COLORS[i] - COLORS[i + 1]) / 2.0 + COLORS[i + 1];
	}
}

int findCircles() {

	// initFindCircles();
	Mat gray;
	Mat canny;

	/*
	  Transform it into the C++ cv::Mat format
	*/
	// image = imread("img.png", 1);

	/*
	  Cria retangulo para filtragem da area de trabalho
	*/
	// Rect box = Rect(300, 57, 330, 400); //Vostro FOV workspace
	Rect box = Rect(300, 50, 330, 400); //LAB FOV workspace
	// Rect box = Rect(320, 157, 277, 200); //Vostro FOV scanner
	// Rect box = Rect(360, 170, 210, 180); //Vostro FOV imagem
	// Rect box = Rect(360, 170, 200, 170);
	// Rect box = Rect(560, 260, 300, 260);

	/*
	  Setup a rectangle to define your region of interest
	*/
	Mat src(image, box);
	//cvtColor(src, src, CV_8U);

	/*
	  Carrega a matriz para o reconhecimento de cores
	*/
	srcHsv = src.clone();;
	medianBlur(srcHsv, srcHsv, 11);
	srcHsv.convertTo(srcHsv, -1, 1.2, -70);
	cvtColor(src, srcHsv, COLOR_BGR2HSV);
	medianBlur(srcHsv, srcHsv, 11);


	if (DEBUG) {
		namedWindow("srcHsv"); imshow("srcHsv", srcHsv);
		moveWindow("srcHsv", OFFSET * windowCont++, 0);
	}

	/*
	  Carrega a matriz para o reconhecimento dos circulos
	*/
	cvtColor(src, gray, CV_RGB2GRAY);
	// cvtColor( src, gray, CV_BGR2GRAY );

	/*
	  Filtragem - Escurecer o amarelo
	*/
	gray.convertTo(gray, -1, 1.3, -200);
	if (DEBUG) {
		namedWindow("gray"); imshow("gray", gray);
		moveWindow("gray", OFFSET * windowCont++, 0);
	}
	/*
	  Filtragem - Escurecer o filtragem pre-canny contornos
	*/
	medianBlur(gray, gray, 3);
	if (DEBUG) {
		namedWindow("blur"); imshow("blur", gray);
		moveWindow("blur", OFFSET * windowCont++, 0);
	}

	/*
	  Filtragem - Cria matriz de contornos
	*/
	Canny(gray, canny, 10, 140, 3, true);
	if (DEBUG) {
		namedWindow("canny1"); imshow("canny1", canny > 0);
		moveWindow("canny1", OFFSET * windowCont++, 0);
	}

	/*
	  Filtragem - Aumenta bordas
	*/
	blur(canny, canny, Size( 3, 3 ), Point(-1, -1));
	if (DEBUG) {
		namedWindow("canny3"); imshow("canny3", canny > 0);
		moveWindow("canny3", OFFSET * windowCont++, 0);
	}

	/*
	  Filtragem - Suaviza imperfeicoes
	*/
	medianBlur(canny, canny, 5);
	if (DEBUG) {
		namedWindow("canny3Blur"); imshow("canny3Blur", canny > 0);
		moveWindow("canny3Blur", OFFSET * windowCont++, 0);
	}
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
	char idCircle;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		idCircle = checkColor(center); //find color
		if (DEBUG)
			cout << "center : " << center << "\tradius : " << radius << "\thue: " << idCircle << "\tcolor: " << (int)srcHsv.at<Vec3b>(center).val[0] << " " << (int)srcHsv.at<Vec3b>(center).val[1] << " " << (int)srcHsv.at<Vec3b>(center).val[2] << endl;
		fprintf(file, "%d %d %c\n", center.x, center.y, idCircle);

		circle( src, center, 3, Scalar(0, 255, 0), -1, 8, 0 ); // circle center
		circle( src, center, radius, Scalar(0, 0, 255), 3, 8, 0 ); // circle outline
	}

	fclose(file);

	// Show your results
	if (DEBUG)
	{
		namedWindow("src"); imshow("src", src);
		moveWindow("src", OFFSET * windowCont++, 0);
	}
	imwrite( "./circulos.png", src );




	// namedWindow("color"); imshow("color", srcHsv);
	// moveWindow("color", 0, 250);

	// waitKey(0);
	return circles.size();
}


char checkColor(Point center) {
	int size = 16;
	int sample[3], hue, saturation;
	char hueDebug;
	for (int j = 0; j < 3; ++j)
	{
		sample[j] = 0;
		for (int i = 0; i < size / 4; ++i)
		{
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(size / 2 - i, i)).val[j];
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(size / 4  - i, -i)).val[j];
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(-size / 4  + i, i)).val[j];
			sample[j] += (int)srcHsv.at<Vec3b>(center + Point(-size / 4  + i, -i)).val[j];

		}
	}
	hue = sample[0] / size;
	if (DEBUG)
		cout << "HSV:" << sample[0] / size << " " << sample[1] / size << " " << sample[2] / size << " - ";
	saturation = sample[2] / size;
	for (int i = 0; i < NUM_OF_COLORS - 1; ++i)
	{
		if (saturation < BALCK_SATURATION)
			return 't'; // tool
		else if (hue > LIMITS[i])
			return ANSWERS[i];
	}
	return ANSWERS[NUM_OF_COLORS - 1];
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