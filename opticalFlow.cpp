//
// Utilises the openCV library. Please see license http://opencv.org/license.html
//

#include "stdafx.h"
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

int pixel_step, channels, pixel_step_out, channels_out;
int xt, yt;
int xout, yout;
int x2, y2;
int i=0, count;
uchar *data_out, *data;
IplImage *out=NULL, *frame;
CvSeq* first_contour, *contours2;						//Define Variables
CvMemStorage* storage = cvCreateMemStorage();	
double Result, Result2;
CvRect rect;

int main(void)
{
	CvCapture *webcam = cvCaptureFromCAM(0);				//Create Capture Variables
	frame=cvQueryFrame(webcam);						//Grab a Frame
	cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);			//Create Output Windows
	cvNamedWindow("Sat", CV_WINDOW_AUTOSIZE);
	while(true) {
		static IplImage *frame = NULL, *frame1 = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;

		frame = cvQueryFrame(webcam);
		//----Setup parameters for cvCalcOpticalFlowPyrLK------//
		frame1_1C = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
		cvConvertImage(frame, frame1_1C, 0);				//Setup Frame(t) variables
		frame1 = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		cvConvertImage(frame, frame1, 0);
		Sleep(33);							//Pause for Period of 1 frame
		frame = cvQueryFrame(webcam);
		frame2_1C = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);	//Setup Frame(t+dt) variables
		cvConvertImage(frame, frame2_1C, 0);
		eig_image = cvCreateImage(cvGetSize(frame), IPL_DEPTH_32F, 1);
		temp_image = cvCreateImage(cvGetSize(frame), IPL_DEPTH_32F, 1);	//Setup Optical Flow Parameters
		CvPoint2D32f frame1_features[15000];
		CvPoint2D32f frame2_features[15000];
		int number_of_features = 15000;
		cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, .01, NULL);
		char optical_flow_found_feature[15000];
		float optical_flow_feature_error[15000];			//Setup Optical Flow Parameters
		CvSize optical_flow_window = cvSize(3,3);
		CvTermCriteria optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
		pyramid1 = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
		pyramid2 = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

		//---Perform Lucas-Kanade Optical Flow Algorithm---//
		cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );
		//-------------------------------------------------//
		//---Initilise Second Image Parameters---//
		out = cvCreateImage( cvGetSize(frame1), 8, 3 );
		pixel_step_out = out->widthStep;
		channels_out = out->nChannels;
		data_out = (uchar *)out->imageData;
		//--------------------------------------//
		xt=0;
		yt=0;
		count=1;

		for (i = 0; i < number_of_features; i++) {

			if ( optical_flow_found_feature[i] != 0 ) {
			CvPoint p,q;
			p.x = (int) frame1_features[i].x;
			p.y = (int) frame1_features[i].y;			//Find Optical Flow Magnitude Points
			q.x = (int) frame2_features[i].x;
			q.y = (int) frame2_features[i].y;
			double calc=((p.x-q.x)^2)+((p.x-q.x)^2);
			double calc2=((p.y-q.y)^2)+((p.y-q.y)^2);		//Calculate Optical Flow Magnitude
			double calc3=(calc*calc)+(calc2*calc2);
			if (sqrt(calc3)>120) {					//Apply Optical Flow Threshold
				data_out[p.y*pixel_step_out+p.x*channels_out]=255;	
				xt=xt+p.x;					//Store X point
				yt=yt+p.y;					//Store Y point
				count=count+1;
			}
			}
		}
		if ( optical_flow_found_feature[i] != 0 ) {
				xout=xt/count;					//Calculate X point
				yout=yt/count;					//Calculate Y point
		}
		else {
			xout=5;
			yout=5;
		}
		cvShowImage("Optical Flow", frame1);				//Output a frame
		cvThreshold(out, out, 128, 255, CV_THRESH_BINARY);
		CvPoint p1 = cvPoint((xout)-5,(yout)-5);			//Draw point on output image
		CvPoint p2 = cvPoint((xout)+5,(yout)+5);
		cvRectangle(out, p1, p2, CV_RGB(200, 0, 200), 10, 8, 0 );
		cvShowImage("Sat", out);
		cvWaitKey(1);							//Wait to see if user presses 'esc'
	}
}
