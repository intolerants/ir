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
#define BLUE 178
#define YELLOW 154
#define BLACK 16
#define NUM_OF_COLORS 3
#define OFFSET 350
#define OFFSETY 350
#define CONTRAST 0.2
#define BRIGHTNESS 20
#define aBLUR 2
#define WIDTH 1200

using namespace std;
using namespace cv;

void takePicture(void);
void initFindCircles(void);
int findCircles(void);
char checkColor(Point center);
void openFile(void);


int COLORS[NUM_OF_COLORS] = {BLUE, YELLOW, BLACK};
char ANSWERS[NUM_OF_COLORS] = {'e', 'b', 'w'}; //end, begin, wall
float LIMITS[NUM_OF_COLORS - 1];
Mat src, srcHsv, image;
int windowCont = -1;
double brightness = -160;
double contrast = 2;
// int ablur = -1;
int ablur = 11;
Point pTemp;

Point windowPos(void) {
	int pos = ++windowCont*OFFSET;
	return Point((pos%WIDTH),(((pos/WIDTH))*OFFSETY));
}

void showWindow(Mat m, string s)
{
		namedWindow(s, WINDOW_FREERATIO);
		imshow(s, m);
		resizeWindow(s, 100,100);
		pTemp = windowPos();
#ifdef DEBUG
	cout << "----" << s << "(" << pTemp.x << " ," << pTemp.y << ")" << endl;
#endif
		moveWindow(s, pTemp.x, pTemp.y);
}

int main(int argc, const char** argv)
{
	initFindCircles();
	// takePicture();
	findCircles();
	return 0;
}

void takePicture() {
	CvCapture* capture = 0;
	Mat frame, frameCopy;

	capture = cvCaptureFromCAM( -1 ); //0=default, -1=any camera, 1..99=your camera

	if (!capture)
	{
#ifdef DEBUG
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

				int numOfCircles = findCircles();
				if ( waitKey( 10 ) >= 0 || numOfCircles == 7)
						break;
				cout << " =========== AGAIN ============\n\n";
			}
			/*IplImage* iplImg = cvQueryFrame(capture);
			cvShowImage("result", iplImg);
			cvSaveImage("img.png", iplImg);
			waitKey(0);*/
		}
		cvReleaseCapture(&capture);
		cvDestroyWindow("result");
	}
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

	while (1) {
		// initFindCircles();
		Mat gray;
		Mat canny;

		/*
		  Transform it into the C++ cv::Mat format
		*/
		image = imread("1eComFundo.jpg", 1);

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


#ifdef DEBUG
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
		char idCircle;
		for (size_t i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);

			idCircle = checkColor(center); //find color
#ifdef DEBUG
			cout << "center : " << center << "\tradius : " << radius << "\thue: " << idCircle << "\tcolor: " << (int)srcHsv.at<Vec3b>(center).val[0] << " " << (int)srcHsv.at<Vec3b>(center).val[1] << " " << (int)srcHsv.at<Vec3b>(center).val[2] << endl;
#endif
			fprintf(file, "%d %d %c\n", center.x, center.y, idCircle);

			circle( src, center, 3, Scalar(0, 255, 0), -1, 8, 0 ); // circle center
			circle( src, center, radius, Scalar(0, 0, 255), 3, 8, 0 ); // circle outline
		}

		fclose(file);

		// Show your results
#ifdef DEBUG
		showWindow(src, "src");
#endif
		imwrite( "./circulos.png", src );

		// namedWindow("color"); imshow("color", srcHsv);
		// moveWindow("color", 0, 250);
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
		windowCont = -1;
		destroyAllWindows();
#ifdef DEBUG
		cout << "contrast:" << contrast << "\tbrightness:" << brightness << "\tblur:" << ablur << endl;
#endif
	}
	return 0;
}


char checkColor(Point center) {
	int size = 120;
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
	// showWindow(srcHsv, "srcHsv");
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