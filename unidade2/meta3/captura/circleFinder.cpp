// Ver em: http://hxr99.blogspot.com.br/2011/12/opencv-examples-camera-capture.html

// -------------------------------------------------------------------------------
//  OpenCV C/C++ Examples (Camera Capture)
// There are also python implementation for camera capture and showing image.
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;


/*
  Cor  : H   S  V
  Verme: 352 41 83
  Rosa : 312 47 77
  Azul : 216 55 89
  Ciano: 203 60 100
  Verde: 153 41 77
  Amare: 56  51 100
  Cor       H     S     V
  Vermelho  352   41    83
            332   44    80
  Rosa      312   47    77
            264   51    83
  Azul      216   55    89
            209,5 57,5  94,5
  Ciano     203   60    100
            178   50,5  88,5
  Verde     153   41    77
            104,5 46    88,5
  Amarelo   56    51    100

  RED 175
  PINK 155
  BLUE 108
  CIAN 103
  GREEN 76
  YELLOW 27
  BLACK 136+26
*/

/*
#define RED 352
#define PINK 312
#define BLUE 216
#define CIAN 203
#define GREEN 153
#define YELLOW 56
*/

#define RED 175
#define PINK 155
#define BLUE 108
#define CIAN 103
#define GREEN 76
#define YELLOW 27

#define BALCK_SATURATION 162

#define NUM_OF_COLORS 6

int COLORS[NUM_OF_COLORS] = {RED, PINK, BLUE, CIAN, GREEN, YELLOW};
char ANSWERS[NUM_OF_COLORS] = {'r', 'p', 'b', 'c', 'g', 'y'};
float LIMITS[NUM_OF_COLORS-1];
Mat src, srcHsv;
void takePicture(void);
void findCircles(void);
char checkColor(Point center);

int main(int argc, const char** argv)
{
  for (int i = 0; i < NUM_OF_COLORS-1; ++i)
  {
    LIMITS[i] = (COLORS[i] - COLORS[i+1])/2.0 + COLORS[i+1];
    // cout << "Limits:" << LIMITS[i] << " Answer:" << ANSWERS[i] << " Colors:" << COLORS[i] << endl;
  }
	// takePicture();
	findCircles();

	return 0;
}


void takePicture() {
  CvCapture* capture = 0;
  Mat frame, frameCopy, image;

  capture = cvCaptureFromCAM( -1 ); //0=default, -1=any camera, 1..99=your camera

  if (!capture)
  {
    cout << "No camera detected" << endl;
  }

  cvNamedWindow("result", CV_WINDOW_AUTOSIZE);

  if (capture)
  {
    cout << "In capture ..." << endl;
    // for (;;)
    // {
    //  IplImage* iplImg = cvQueryFrame( capture );
    //  frame = iplImg;

    //  if ( frame.empty() )
    //    break;
    //  if ( iplImg->origin == IPL_ORIGIN_TL )
    //    frame.copyTo( frameCopy );
    //  else
    //    flip( frame, frameCopy, 0 );

    //  cvShowImage( "result", iplImg );

    //  if ( waitKey( 10 ) >= 0 )
    //    break;
    // }
    IplImage* iplImg = cvQueryFrame(capture);
    cvShowImage("result", iplImg);
    cvSaveImage("img.png", iplImg);
    //waitKey(0);
  }

  
  cvReleaseCapture(&capture);
  cvDestroyWindow("result");
}

void findCircles() {
  Mat gray;
  Mat canny;

  /*
    Transform it into the C++ cv::Mat format
  */
  Mat image = imread("img.png", 1);

  /*
    Cria retangulo para filtragem da area de trabalho
  */
  Rect box = Rect(360, 170, 210, 180);
  // Rect box = Rect(360, 170, 200, 170);
  // Rect box = Rect(560, 260, 300, 260);
  
  /*
    Setup a rectangle to define your region of interest
  */
  Mat src(image,box);

  /*
    Carrega a mamtriz para o reconhecimento de cores
  */
  cvtColor(src, srcHsv, CV_BGR2HSV);

  /*
    Carrega a matriz para o reconhecimento dos circulos
  */
  cvtColor(src, gray, CV_RGB2GRAY);
  // cvtColor( src, gray, CV_BGR2GRAY );
  
  /*
    Filtragem - Escurecer o amarelo
  */
  gray.convertTo(gray, -1, 1.3, -200);
  // namedWindow("gray"); imshow("gray", gray);
  // moveWindow("gray", 270, 0);

  /*
    Filtragem - Escurecer o filtragem pre-canny contornos 
  */
  medianBlur(gray, gray, 3);
  // namedWindow("blur"); imshow("blur", gray);
  // moveWindow("blur", 270, 300);

  /*
    Filtragem - Cria matriz de contornos 
  */
  Canny(gray, canny, 10, 140, 3, true);
  // namedWindow("canny1"); imshow("canny1", canny > 0);
  // moveWindow("canny1", 270*2, 0);

  /*
    Filtragem - Aumenta bordas
  */
  blur(canny, canny, Size( 3, 3 ), Point(-1,-1));

  // namedWindow("canny3"); imshow("canny3", canny > 0);
  // moveWindow("canny3", 270*4, 0);

  /*
    Filtragem - Suaviza imperfeicoes
  */
  medianBlur(canny, canny, 5);

  // namedWindow("canny3Blur"); imshow("canny3Blur", canny > 0);
  // moveWindow("canny3Blur", 270*4, 250);
  // moveWindow("canny3", 720+220, 0);
 


  /*
    Apply the Hough Transform to find the circles
  */
  vector<Vec3f> circles;
  HoughCircles(canny, circles, CV_HOUGH_GRADIENT, 0.5, 30, 200, 50, 0, 0);
 
  /*
    Draw the circles detected
  */
  for(size_t i = 0; i < circles.size(); i++)
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);     
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );// circle center     
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
      cout << "center : " << center << "\tradius : " << radius << "\thue: " << checkColor(center) << "\tcolor: " << (int)srcHsv.at<Vec3b>(center).val[0] << " " << (int)srcHsv.at<Vec3b>(center).val[1] << " " << (int)srcHsv.at<Vec3b>(center).val[2] << endl;
      
   }
 
  // Show your results
  namedWindow("src"); imshow("src", src);
  moveWindow("src", 0, 0);


  // namedWindow("color"); imshow("color", srcHsv);
  // moveWindow("color", 0, 250);

  waitKey(0);
}


char checkColor(Point center) {
  int size = 24;
  int sample[3], hue, saturation;
  char hueDebug;
  for (int j = 0; j < 3; ++j)
  {
    sample[j] = 0;
    for (int i = 0; i < size/4; ++i)
    {
      sample[j] += (int)srcHsv.at<Vec3b>(center + Point(size/2 - i, i)).val[j];
      sample[j] += (int)srcHsv.at<Vec3b>(center + Point(size/4  - i, -i)).val[j];
      sample[j] += (int)srcHsv.at<Vec3b>(center + Point(-size/4  + i, i)).val[j];
      sample[j] += (int)srcHsv.at<Vec3b>(center + Point(-size/4  + i, -i)).val[j];

    }
  }
    hue = sample[0]/size;
    saturation = sample[2]/size;
  for (int i = 0; i < NUM_OF_COLORS-1; ++i)
  {
    if (saturation < BALCK_SATURATION)
      return 't'; // tool
    else if (hue > LIMITS[i])
      return ANSWERS[i];
  }
  return ANSWERS[NUM_OF_COLORS-1];
}
