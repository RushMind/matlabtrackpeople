#include "highgui.h"
#include "cv.h"
#include "cvaux.h"
#include <iostream>
#include "stdlib.h"
#include <vector>
#include <algorithm>
using namespace std;
const int CONTOUR_MAX_AERA=300;//块阈值
const int N=20;//用来估计背景的帧数

void bgestimator(IplImage* background,CvCapture* video)
{
	CvSize videosize;
	videosize.width=cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_WIDTH);
	videosize.height=cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_HEIGHT);
	int step=videosize.width*videosize.height;
	IplImage* bgtosort=cvCreateImage(cvSize(step,N),IPL_DEPTH_8U,1);
	IplImage* pframe=0;
	IplImage* grayimg=cvCreateImage(videosize,IPL_DEPTH_8U,1);
	int i=0,j=0,k=0;
	//得到N帧的像素，并reshape
	while(i<N)
	{
		pframe=cvQueryFrame(video);
		cvCvtColor(pframe,grayimg,CV_BGR2GRAY);		
		for(j=0;j<grayimg->height;j++)
			for(k=0;k<grayimg->width;k++)
			{
				int p=j*grayimg->widthStep+k;
				uchar* curr_data=(uchar*)(grayimg->imageData)+j*grayimg->widthStep+k;
				uchar* bgtosort_data=(uchar*)(bgtosort->imageData)+i*bgtosort->widthStep+p;
				*bgtosort_data=*curr_data;
			}
			
			i++;
	}
	//去像素的中间值
	uchar* sort_vector=new uchar[N];
	uchar* temp=new uchar[step];
	for(i=0;i<bgtosort->width;i++)
	{	//cout<<"processing"<<i<<endl;	
		for(j=0;j<bgtosort->height;j++)
		{
			uchar* sort_data=(uchar*)(bgtosort->imageData)+j*bgtosort->widthStep+i;
			sort_vector[j]=*sort_data;

		}
		sort(sort_vector,sort_vector+N);
		temp[i]=sort_vector[N/2];
	}
	//获得背景图
	for(i=0;i<background->height;i++)
		for(j=0;j<background->width;j++)
		{
			uchar* bgdata=(uchar*)(background->imageData)+i*background->widthStep+j;
			*bgdata=temp[i*background->width+j];
		}
		cvReleaseImage(&bgtosort);
}

void main()
{
	CvCapture* video=cvCaptureFromFile("walk.avi");
	IplImage* pframe=0;
	CvSize vsize;
	int nframe;
	vsize.width=cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_WIDTH);
	vsize.height=cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_HEIGHT);
	IplImage* bg=cvCreateImage(vsize,IPL_DEPTH_8U,1);
	IplImage* grayimg=cvCreateImage(vsize,IPL_DEPTH_8U,1);
	IplImage* fgimg=cvCreateImage(vsize,IPL_DEPTH_8U,1);
	IplImage* temp=cvCreateImage(vsize,IPL_DEPTH_8U,1);
	IplConvKernel* strelv=cvCreateStructuringElementEx(1,11,0,4,CV_SHAPE_RECT);
	IplConvKernel* strelh=cvCreateStructuringElementEx(5,1,3,0,CV_SHAPE_RECT);
	//estimate the background image
	bgestimator(bg,video);
	cvNamedWindow("background");
	cvMoveWindow("background",370,0);
	cvShowImage("background",bg);
	cvNamedWindow("foreground");
	cvMoveWindow("foreground",200,0);
	cvNamedWindow("video");
	cvMoveWindow("video",30,0);

	CvFont font;
	int flag=0;
    cvInitFont( &font,CV_FONT_HERSHEY_SIMPLEX,0.5, 0.5, 0,1, 8);
	vector<CvRect> srect;
	while(pframe=cvQueryFrame(video))
	{
		++flag;
		CvMemStorage *stor;
		stor = cvCreateMemStorage(0);
	    CvSeq* cont,tmpcont;
        cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
		cvCvtColor(pframe,grayimg,CV_BGR2GRAY);
		cvAbsDiff(grayimg,bg,fgimg);
		cvThreshold(fgimg,fgimg,45,255,CV_THRESH_BINARY);

		//形态学操作，连通相近的块
        cvMorphologyEx(fgimg,fgimg,temp,strelv,CV_MOP_CLOSE);
		cvMorphologyEx(fgimg,fgimg,temp,strelh,CV_MOP_CLOSE);
		cvShowImage("foreground",fgimg);
		int i=1;
		 cvFindContours( fgimg, stor, &cont, sizeof(CvContour), 
                    CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));//只提取最外层的轮廓 
        vector<CvRect> trect;
		for(;cont;cont=cont->h_next)
		 {
			 CvRect r=((CvContour*)cont)->rect;
			 trect.push_back(r);
		 }
		 vector<CvRect>::iterator iter=trect.begin();
		 //合并相近的块
		 while(iter!=trect.end())
		 {
			 vector<CvRect>::iterator niter=iter;
			 ++niter;
			 while(niter!=trect.end())
			 {
			 int max_top=max(iter->y,niter->y);
			 int max_left=max(iter->x,niter->x);
			 int min_bottom=min(iter->y+iter->height,niter->y+niter->height);
			 int min_right=min(iter->x+iter->width,niter->x+niter->width);
			 int min_top=min(iter->y,niter->y);
			 int min_left=min(iter->x,niter->x);
			 int max_bottom=max(iter->y+iter->height,niter->y+niter->height);
			 int max_right=max(iter->x+iter->width,niter->x+niter->width);
			 if(max_top-min_bottom<10&&max_left-min_right<10&&(niter->width*niter->height<400||iter->width*iter->height<400))
			 {
				 iter->y=min_top;
				 iter->x=min_left;
				 iter->height=max_bottom-min_top;
				 iter->width=max_right-min_left;
				 niter=trect.erase(niter);
			 }
			 else 
				 ++niter;
			 }
			 //去掉小块
			 if(iter->width*iter->height<CONTOUR_MAX_AERA)
				 iter=trect.erase(iter);
			 else
			 ++iter;
		 }
		 //给检测到的方框标号
		 
		  //检测结果发生变化，显示检测结果		 
		 if(srect.size()!=trect.size()&&flag!=1)
		 {
		 if(trect.size()==0)cout<<"no target was found!"<<endl;
		 else
			 if(trect.size()==1)
				 cout<<"1 target was found!"<<endl;
		 else
		 cout<<trect.size()<<" targets were found"<<endl;
		 } 
		 for(iter=trect.begin();iter!=trect.end();++iter)
		 {
//			 cout<<"area "<<i<<" is "<<iter->height*iter->width<<endl;
			 if(iter->height * iter->width > CONTOUR_MAX_AERA) // 面积小的方形抛弃掉
			 {
			  cvRectangle( pframe, cvPoint(iter->x,iter->y), 
                          cvPoint(iter->x + iter->width, iter->y + iter->height),
                          CV_RGB(255,0,0), 1, CV_AA,0);
				  char  str[3];			
				  cvPutText(pframe,itoa(i,str,10),cvPoint(iter->x,iter->y+12),&font,CV_RGB(255,0,0));
				  ++i;
			 }
		 }
		 srect.assign(trect.begin(),trect.end());
    // free memory      
		cvShowImage("video",pframe);
		cvReleaseMemStorage(&stor);
		char k = cvWaitKey(20);
        if( k == 27 ) break;
	}
	cvReleaseCapture(&video);
	cvReleaseImage(&bg);
	cvReleaseImage(&grayimg);
	cvReleaseImage(&temp);
    cvReleaseImage(&fgimg);
}