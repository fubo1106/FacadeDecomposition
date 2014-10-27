
// ImageProcessingDlg.cpp : implementation file
//

#include "stdafx.h"
#include "ImageProcessing.h"
#include "ImageProcessingDlg.h"
#include "RectangleTools.h"
#include "afxdialogex.h"
#include "Utilities.h"

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <queue>

#define MAX_LINES 25000 //max number of lines in the image
#define MIN_LENGTH 20   //min length of a line segment
//#define MIN_DISTANCE 8 //min distance between 2 segments,the threshold to decide whether merge them or not
//#define	THRESH_RECT_LENGTH  25 //min threshold that 2 segments can be 2 edges of a rectangle 
//#define THRESH_RECT_VERTICAL 0.05   //min threshold that 2 segments can be 2 adjacent edges of a rectangle 
//#define THRESH_RECT_PARALLEL 0.95 
//#define THRESG_RECT_CENTER 0 //threshold for whether a point at(x,y) is the center for a rectangle
//#define MIN_RECT_EDGELENGTH 10 //min radius
//#define MAX_RECT_EDGELENGTH 250
//#define IMAGE_SCALE 1
#define CLOCKS_PER_SEC ((clock_t)1000) 

using namespace std;
using namespace cv;

/*head filesfile operation*/
#include <io.h>
#include <fstream>
#include <fcntl.h>//Rain

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

void InitConsoleWindow()//在Image_Dlg中调用
{
	int nCrt = 0;
	FILE* fp;
	AllocConsole();
	nCrt = _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	fp = _fdopen(nCrt, "w");
	*stdout = *fp;
	setvbuf(stdout, NULL, _IONBF, 0);
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CImageProcessingDlg dialog



CImageProcessingDlg::CImageProcessingDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CImageProcessingDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CImageProcessingDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CImageProcessingDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_Linedetection, &CImageProcessingDlg::OnBnClickedLinedetection)
	ON_BN_CLICKED(IDC_decomposition, &CImageProcessingDlg::OnBnClickeddecomposition)
	ON_BN_CLICKED(IDC_equalizeHist, &CImageProcessingDlg::OnBnClickedequalizehist)
	ON_BN_CLICKED(IDC_averageDepth, &CImageProcessingDlg::OnBnClickedaveragedepth)
END_MESSAGE_MAP()


// CImageProcessingDlg message handlers

BOOL CImageProcessingDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	InitConsoleWindow();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CImageProcessingDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CImageProcessingDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CImageProcessingDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

Vec4i getDistictSegment(vector<Vec4i>& v){
	
	if(v.size()==1)
		return Vec4i(v[0][0],v[0][1],v[0][2],v[0][3]);

	vector<Point> vv;
	vector<double>vx,vy;
	Point p1,p2;
	bool flag_vertival = false;
	double MAX_SLOPE = 10;
	for(int i=0;i<v.size();i++){
		p1 = Point(v[i][0],v[i][1]);
		p2 = Point(v[i][2],v[i][3]);
		if(p1.x == p2.x || abs((p2.y-p1.y)/(p2.x-p1.x)) >= MAX_SLOPE)
			flag_vertival = true;
		vv.push_back(p1);
		vv.push_back(p2);

		vx.push_back(p1.x);
		vx.push_back(p2.x);

		vy.push_back(p1.y);
		vy.push_back(p2.y);
	}
	double start_x,start_y,end_x,end_y;
	std::sort(vx.begin(),vx.end());

	if(!flag_vertival){
		Mat A = get_ls_value(vv);
	
		double k = A.at<double>(0,0);
		double b = A.at<double>(1,0);	
		//std::sort(vy.begin(),vy.end());
		start_x = vx[0];
		end_x = vx[vx.size()-1];

		start_y = k * start_x + b;
		end_y = k * end_x + b;
	}
	else{
		std::sort(vy.begin(),vy.end());
		start_x = vx[0];
		end_x = vx[0];
		start_y = vy[0];
		end_y = vy[vy.size()-1];
	}
	return Vec4i(start_x,start_y,end_x,end_y);
}

vector<int> findRelatedLines(Vec4i line,vector<Vec4i>lineSet){
	vector<int> a;
	for(int i=0;i<lineSet.size();i++){
		if(isSameLine(line,lineSet[i]) && lineSet[i] != line)
			a.push_back(i);
	}
	return a;
}

void segmentsTolines(vector<Vec4i>& segments, vector<Vec4i>* lines){
	/*合并线段segments*/
	vector<Line>lineSet;
	int num = segments.size();

	/*Vec4i pp(797,722,798,670);
	Vec4i pp1(798,758,798,728);
	vector<Vec4i>tempLineSet;
	boolean b = isSameLine(pp,pp1);
	vector<int>finded = findRelatedLines(pp,segments);
	for(int nn=0;nn<finded.size();nn++)
		tempLineSet.push_back(segments[finded[nn]]);
	Vec4i ss = getDistictSegment(tempLineSet);*/

	//Vec4i s = getDistictSegment(lineSet);
#if 0
	/*use num queues to do the BFS*/
	queue<Vec4i>* pQueue[MAX_LINES];
	//initialization each queue contains one element
	if(MAX_LINES < num){
		cout<<"too many lines: exceed "<<MAX_LINES<<endl;
		exit(0);
	}
	else{
		for(int i=0;i<num;i++){
			pQueue[i] = new queue<Vec4i>;
			pQueue[i]->push(segments[i]);
		}
	}

	int VISITED = 1;
	int mark[MAX_LINES];

	for(int i=0;i<num;i++){
		mark[i] = -1;//initialization marks
	}

	for(int i=0;i<num;i++){
		while(!pQueue[i]->empty()){
			Vec4i p = pQueue[i]->front();
			pQueue[i]->pop();

			vector<int> relatedLines = findRelatedLines(p,segments);

			for(int j=0;j<relatedLines.size();j++){
				if(!mark[relatedLines[j]]){
					pQueue[i]->push(segments[relatedLines[j]]);
					pQueue[relatedLines[j]]->pop();
					mark[relatedLines[j]] = VISITED;
				}
			}
		}
	}
#else 
	queue<Vec4i> pQueue;
	vector<Vec4i> classSegments;
	
	if(MAX_LINES < num){
		cout<<"too many lines: exceed "<<MAX_LINES<<endl;
		exit(0);
	}
	else{
		int mark[MAX_LINES];

		for(int i=0;i<num;i++){
			mark[i] = -1;//initialization marks
		}
		for(int i=0;i<num;i++){
			if(mark[i] == -1){
				pQueue.push(segments[i]);
				classSegments.push_back(segments[i]);

				while(!pQueue.empty()){
					Vec4i p = pQueue.front();
					pQueue.pop();

					vector<int> relatedLines = findRelatedLines(p,segments);

					for(int j=0;j<relatedLines.size();j++){
						if(mark[relatedLines[j]] == -1){
							pQueue.push(segments[relatedLines[j]]);
							classSegments.push_back(segments[relatedLines[j]]);
							mark[relatedLines[j]] = 1;
						}
					}
				}
				Vec4i s = getDistictSegment(classSegments);
				if(distance1(Point(s[0],s[1]),Point(s[2],s[3])) >= MIN_LENGTH * IMAGE_SCALE)
					lines->push_back(s);
				/*lines could be a class*/
				/*Mat orig;
				cv::cvtColor(dst,orig,CV_GRAY2BGR);
				for(int kk = 0;kk<classSegments.size();kk++){
					cout<<"segment:"<<segments[i]<<",it's set:"<<classSegments[kk]<<endl;
					cv::line(orig,Point(classSegments[kk][0],classSegments[kk][1]),Point(classSegments[kk][2],classSegments[kk][3]),Scalar(0,255,0),3,CV_AA);
				}
				cv::line(orig,Point(s[0],s[1]),Point(s[2],s[3]),Scalar(0,0,255),2,CV_AA);
				cv::namedWindow("big_line",0);
				cv::imshow("big_line",orig);
				cv::waitKey(0);*/

				classSegments.clear();
			}

		}
	}
#endif
	return;
	//q[0].push(segments[0]);
	//mark[0] = VISITED;
	//while(!q[0].empty()){
	//	Vec4i f= q[0].front();
	//	q[0].pop();
	//	vector<Vec4i>next = findNext(f);
	//	for(int i=0;i<next.size();i++){
	//		if(!VISITED) push(next[i]); VISITED;
	//	}
	//}
	//makeSet(segments,&lineSet);
	/*for(int i=0;i<lineSet.size()-1;i++){
		mergeSet(lineSet[1],lineSet[i+1]);
	}*/
	/*vector<Line>newSet; 
	int start = 0;
	int end = lineSet.size()-1;
	merge(lineSet,start,end);*/
	/*for(int i=0;i<segments.size();i++){
		line = segments[i];
		start = Point(line[0],line[1]);
		end = Point(line[2],line[3]);
	}*/
}

boolean isInsideRing(Vec4i& line,int x,int y,int rmin,int rmax){
	Point p1(line[0],line[1]);
	Point p2(line[2],line[3]);
	Point center(x,y);

	if(distance1(center,p1)>rmin && distance1(center,p1)<rmax && distance1(center,p2)>rmin && distance1(center,p2)<rmax)
		return true;
	return false;
}

boolean isCenterAtPix(Segment& s1,Segment& s2,Segment& s3,Segment& s4,int center_x,int center_y){
	//double t1 = clock();
	vector<Point> v;

	Point p10 = s1.startp;
	Point p11  =s1.endp;

	Point p20 = s2.startp;
	Point p21 = s2.endp;

	Point p30 = s3.startp;
	Point p31 = s3.endp;

	Point p40 = s4.startp;
	Point p41 = s4.endp;

	v.push_back(p10);
	v.push_back(p11);
	v.push_back(p20);
	v.push_back(p21);
	v.push_back(p30);
	v.push_back(p31);
	v.push_back(p40);
	v.push_back(p41);
	
	Point centerP = calcCenterPoint(v);
	double t2 = clock();
	//cout<<"isCenterAtPix:"<<(t2-t1)/CLOCKS_PER_SEC<<"sec"<<endl;
	if( abs(centerP.x - center_x) <= THRESG_RECT_CENTER && abs(centerP.y - center_y) <= THRESG_RECT_CENTER)
		return true;
	return false;
}

vector<Segment> findLinesAtPix(vector<Vec4i>& segments,int x,int y,int rmin,int rmax){
	/*find all the lines at (x,y)*/
	vector<Segment> v;
	Segment s;
	for(int i=0;i<segments.size();i++){
		if(isInsideRing(segments[i],x,y,rmin,rmax)){
			s.startp = Point(segments[i][0],segments[i][1]);
			s.endp = Point(segments[i][2],segments[i][3]);
			v.push_back(s);
		}	
	}
	return v;
}

vector<Rec> findRectFromSegments(vector<Segment>& lines,int center_x,int center_y){
	/*traverse all the line from lines to find all the potential rectangles
	probably there has no rectangle return*/

	/*test code*/
	//Mat displayImage = Mat::zeros(1000,1000,CV_32FC3);
	//Segment s1,s2,s3,s4;
	//vector<Segment> seg;
	//vector<Point>points;
	//

	////344 100 415 100
	//s1.startp = Point(489, 202);s1.endp = Point(559, 199);
	//s2.startp = Point(144, 184);s2.endp = Point(144, 343);
	//s3.startp = Point(344, 199);s3.endp = Point(406, 198);
	//s4.startp = Point(400, 192);s4.endp = Point(400, 343);

	//seg.push_back(s1);seg.push_back(s2);
	//seg.push_back(s3);seg.push_back(s4);

	//points.push_back(s1.startp);points.push_back(s1.endp);
	//points.push_back(s2.startp);points.push_back(s2.endp);
	//points.push_back(s3.startp);points.push_back(s3.endp);
	//points.push_back(s4.startp);points.push_back(s4.endp);

	//Point center = calcCenterPoint(points);
	//

	//boolean a = isRectangle(s1,s2,s3,s4);
	//
	//vector<Segment>segs = getRectSegments(s1,s2,s3,s4);

	//line(displayImage,s1.startp,s1.endp,Scalar(255,0,0),3,CV_AA);
	//line(displayImage,s2.startp,s2.endp,Scalar(0,255,0),3,CV_AA);
	//line(displayImage,s3.startp,s3.endp,Scalar(0,0,255),3,CV_AA);
	//line(displayImage,s4.startp,s4.endp,Scalar(255,255,0),3,CV_AA);

	//namedWindow("test",0);
	//imshow("test",displayImage);
	//cv::waitKey(0);

	vector<Rec> v;
	Rec rec;
	int t = 1;
	for(int i=0;i<lines.size()/t;i++)
		for(int j=i;j<lines.size()/t;j++)
			for(int k=j;k<lines.size()/t;k++)
				for(int l=k;l<lines.size()/t;l++){
						if(isRectangle(lines[i],lines[j],lines[k],lines[l])){
							if(isCenterAtPix(lines[i],lines[j],lines[k],lines[l],center_x,center_y))
							{
								rec.center = Point(center_x,center_y);
								vector<Segment>seg = getRectSegments(lines[i],lines[j],lines[k],lines[l]);
								rec.s1 = seg[0];
								rec.s2 = seg[1];
								rec.s3 = seg[2];
								rec.s4 = seg[3];
								v.push_back(rec);
							}
						}
					
				}

	return v;

}

vector<Rec> duplicateRect(vector<Rec>& rects){
	vector<Rec> dupRec = rects;
	return dupRec;
}

vector<Rec> findRectanglesFromSegments(const Mat& src,vector<Vec4i>& segments,int min,int max){

	/*test code*/
	/*Mat displayImage = Mat::zeros(1000,1000,CV_32FC3);
	Segment s11,s22,s33,s44;
	vector<Segment> seg;
	vector<Point>pointss;
	s11.startp = Point(16, 583);s11.endp = Point(846, 581);
	s22.startp = Point(24, 76);s22.endp = Point(847, 87);
	s33.startp = Point(287, 621);s33.endp = Point(287, 568);
	s44.startp = Point(672, 58);s44.endp = Point(672, 86);

	seg.push_back(s11);seg.push_back(s22);
	seg.push_back(s33);seg.push_back(s44);

	pointss.push_back(s11.startp);pointss.push_back(s11.endp);
	pointss.push_back(s22.startp);pointss.push_back(s22.endp);
	pointss.push_back(s33.startp);pointss.push_back(s33.endp);
	pointss.push_back(s44.startp);pointss.push_back(s44.endp);

	Point center = calcCenterPoint(pointss);
	vector<Segment> segs = findLinesAtPix(segments,center.x,center.y,min,max);

	boolean a = isRectangle(s11,s22,s33,s44);
	
	line(displayImage,s11.startp,s11.endp,Scalar(255,0,0),3,CV_AA);
	line(displayImage,s22.startp,s22.endp,Scalar(0,255,0),3,CV_AA);
	line(displayImage,s33.startp,s33.endp,Scalar(0,0,255),3,CV_AA);
	line(displayImage,s44.startp,s44.endp,Scalar(255,255,0),3,CV_AA);

	namedWindow("test",0);
	imshow("test",displayImage);
	cv::waitKey(0);*/

#if 0 //prevoius rectangle detection approach
	vector<Rec> potentialRects,duplicatedRects;
	vector<Segment> lines;
	for(int i=0;i<src.rows;i++){
		for(int j=0;j<src.cols;j++){
			//double s = clock();
			lines = findLinesAtPix(segments,j,i,min,max);
			if(lines.size()>=4){
				vector<Rec> rect = findRectFromSegments(lines,j,i);
				for(int k=0;k<rect.size();k++){
					potentialRects.push_back(rect[k]);
				}
			}
			/*if(lines.size() > 10)
				cout<<"findRectanglesFromSegments:"<<"findedLine:"<<lines.size()<<"time:"<<(clock()-s)/CLOCKS_PER_SEC<<"seconds"<<endl;*/
		}
	}
	duplicatedRects = duplicateRect(potentialRects);
	return duplicatedRects;
#else
	vector<Rec> potentialRects;
	vector<Point>points;
	Rec rect;
	Segment s1,s2,s3,s4;
	for(int i=0;i<segments.size()-3;i++)
		for(int j=i+1;j<segments.size()-2;j++)
			for(int k=j+1;k<segments.size()-1;k++)
				for(int l=k+1;l<segments.size();l++){
					Point p11(segments[i][0],segments[i][1]);Point p12(segments[i][2],segments[i][3]);
					Point p21(segments[j][0],segments[j][1]);Point p22(segments[j][2],segments[j][3]);
					Point p31(segments[k][0],segments[k][1]);Point p32(segments[k][2],segments[k][3]);
					Point p41(segments[l][0],segments[l][1]);Point p42(segments[l][2],segments[l][3]);
					s1.startp = p11;s1.endp = p12;
					s2.startp = p21;s2.endp = p22;
					s3.startp = p31;s3.endp = p32;
					s4.startp = p41;s4.endp = p42;

					points.push_back(p11);points.push_back(p12);
					points.push_back(p21);points.push_back(p22);
					points.push_back(p31);points.push_back(p32);
					points.push_back(p41);points.push_back(p42);
				
					//vector<Segment> microSeg = RectangleTools::decomposeSegments(s1,s2,s3,s4);
					Rec rect = findRectangle(s1,s2,s3,s4);
					
					if(!existSameRect(rect,potentialRects) && rect.center.x != 0){
						potentialRects.push_back(rect);
					}	
				}
				
					

	return potentialRects;
#endif
	
}

void detectLines(CString filename){
	//char* _path=(LPSTR)(LPCTSTR) filename;
	string name = filename.GetBuffer(0);
	Mat src = imread(name, 1);
	String path1_hough="detectedlines\\"+name+"_hough.txt";
	String path2_hough="detected\\"+name+"_hough.jpg";
	String path3_hough="detected\\"+name+"_hough_showlines.jpg";

	String path1_merge="detectedlines\\"+name+"_merge.txt";
	String path2_merge="detected\\"+name+"_merge.jpg";
	String path3_merge="detected\\"+name+"_merge_showlines.jpg";
	String path4_merge="detected\\"+name+"_merge_src.jpg";
	String path5_rect="detectedlines\\"+name+"_rect.txt";

	String path2_rect="detected\\"+name+"_detectedRectangles.jpg";

	cout<<path1_hough<<endl<<path2_hough<<endl;
	ofstream out_hough(path1_hough),out_merge(path1_merge),out_rect(path5_rect);
	 if(src.empty())
	 {
		 cout << "can not open " << name << endl;
		 return;
	 }

	 Mat dst, cdst, background,src_line,src_rect;

	 /*Canny(src, dst, 50, 200, 3);
	 cvtColor(dst, cdst, CV_GRAY2BGR);*/

	 cvtColor(src,dst,CV_BGR2GRAY);
	 cdst = src.clone();

	 // dst = src;
	 background = Mat::zeros(src.rows,src.cols,CV_32FC3);

	 #if 0
	  vector<Vec2f> lines;
	  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

	  for( size_t i = 0; i < lines.size(); i++ )
	  {
		 float rho = lines[i][0], theta = lines[i][1];
		 Point pt1, pt2;
		 double a = cos(theta), b = sin(theta);
		 double x0 = a*rho, y0 = b*rho;
		 pt1.x = cvRound(x0 + 1000*(-b));
		 pt1.y = cvRound(y0 + 1000*(a));
		 pt2.x = cvRound(x0 - 1000*(-b));
		 pt2.y = cvRound(y0 - 1000*(a));
		 line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
	  }
	 #else
	  vector<Vec4i> lines,merge_lines,dst_lines;
	  HoughLinesP(dst, lines, 1, CV_PI/180, 20, MIN_LENGTH, 20 );
	 // HoughLinesP(dst, lines, 1, CV_PI/180, 8, 6  , 6);
	  /*hough detection visualization part*/
	  cout<<"hough line number:"<<lines.size()<<endl;
	  for( size_t i = 0; i < lines.size(); i++ )
	  {
		Vec4i l = lines[i];
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, CV_AA);
		line( background, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, CV_AA);
		out_hough<<"line"<<i<<":"<<l[0]<<" "<<l[1]<<" "<<l[2]<<" "<<l[3]<<endl;
	  }
	 imwrite(path2_hough,cdst);
	 imwrite(path3_hough,background);

	  /*our merge segments detection visualization part*/
	/*  cvtColor(dst, cdst, CV_GRAY2BGR);
	  background = Mat::zeros(src.rows,src.cols,CV_32FC3);*/
	  double start_merge,end_merge;
	  start_merge = clock();

	  segmentsTolines(lines,&merge_lines);

	 // removeNonVertHorizonSegments(merge_lines,dst_lines);

	  end_merge =clock();
	  cout<<"merge time"<<(end_merge-start_merge)/CLOCKS_PER_SEC<<"seconds"<<endl;

	  cout<<"final segments number:"<<dst_lines.size()<<endl;
	  Scalar color;
	  src_line = src.clone();
	  background = Mat::zeros(src.rows,src.cols,CV_32FC3);
	  for( size_t i = 0; i < dst_lines.size(); i++ )
	  {
		Vec4i l = dst_lines[i];
		if(i%5==0)color = Scalar(255,0,0);
		if(i%5==1)color = Scalar(0,255,0);
		if(i%5==2)color = Scalar(0,0,255);
		if(i%5==3)color = Scalar(255,0,255);
		if(i%5==4)color = Scalar(0,255,255);
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), color, 3, CV_AA);
		line( background, Point(l[0], l[1]), Point(l[2], l[3]), color, 3, CV_AA);
		line( src_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, CV_AA);
		out_merge<<"line"<<i<<":"<<l[0]<<" "<<l[1]<<" "<<l[2]<<" "<<l[3]<<endl;
	  }
	 #endif
	 //imshow("source", src);
	 //imshow("detected lines", cdst);

	 /*imwrite(path2_merge,cdst);
	 imwrite(path3_merge,background);
	 imwrite(path4_merge,src_line);

	 cv::namedWindow("detected Lines",0);
	 cv::imshow("detected Lines",background);
	 cvWaitKey(0);*/
	 src_rect = src.clone();
	 double start_rectDetection,end_rectDetection;
	 start_rectDetection = clock();

	 vector<Rec> rects = findRectanglesFromSegments(src,dst_lines,MIN_RECT_EDGELENGTH * IMAGE_SCALE,MAX_RECT_EDGELENGTH * IMAGE_SCALE);
	 
	 end_rectDetection = clock();
	 cout<<"rectangle detection time"<<(end_rectDetection-start_rectDetection)/CLOCKS_PER_SEC<<"seconds"<<endl;

	 Segment s1,s2,s3,s4;
	 for(int i=0;i<rects.size();i++){
		 s1 = rects[i].s1;
		 s2 = rects[i].s2;
		 s3 = rects[i].s3;
		 s4 = rects[i].s4;

		 line( src_rect, s1.startp, s1.endp, Scalar(0,255,0), 2, CV_AA);
		 line( src_rect, s2.startp, s2.endp, Scalar(0,255,0), 2, CV_AA);
		 line( src_rect, s3.startp, s3.endp, Scalar(0,255,0), 2, CV_AA);
		 line( src_rect, s4.startp, s4.endp, Scalar(0,255,0), 2, CV_AA);
		 drawcross(src_rect,rects[i].center,Scalar(0,0,255),2);
		 out_rect<<"rect"<<i<<"s1:"<<s1.startp<<" "<<s1.endp<<endl;
		 out_rect<<"rect"<<i<<"s2:"<<s2.startp<<" "<<s2.endp<<endl;
		 out_rect<<"rect"<<i<<"s3:"<<s3.startp<<" "<<s3.endp<<endl;
		 out_rect<<"rect"<<i<<"s4:"<<s4.startp<<" "<<s4.endp<<endl;

		 /*namedWindow("detectedRect",0);
		 imshow("detectedRect",src_rect);
		 cv::waitKey(0);
		 src_rect = src.clone();*/
	 }
	 /*namedWindow("detected Rectangles",0);
	 imshow("detected Rectangles",src_rect);
	 cv::waitKey(0);*/
	 imwrite(path2_rect,src_rect);
	 /*namedWindow("detected Rectangles",0);
	 imshow("detected Rectangles",src_rect);
	 cv::waitKey(0);*/
	 return;
}

void CImageProcessingDlg::OnBnClickedLinedetection()
{
	// TODO: Add your control notification handler code here
		/*Point p0(0,0);
	Vec4i seg(-5,1,2,10);
	cout<<distPoint2Segment(p0,seg)<<endl;*/
	/*Vec4i v1(1329,151,1384,151);
	Vec4i v2(1297,112,1393,113);
	if(isSameLine(v1,v2))
		cout<<"is a same line"<<endl;
	else
		cout<<"not the same line"<<endl;*/
	//vector<Vec4i>lineSet;
	//lineSet.push_back(Vec4i(868,441,868,360));
	//lineSet.push_back(Vec4i(866,439,866,398));
	//lineSet.push_back(Vec4i(865,424,866,362));
	//lineSet.push_back(Vec4i(868,441,868,360));
	////lineSet.push_back(Vec4i(583,589,583,510));

	//Vec4i s = getDistictSegment(lineSet);
	//cout<<"distinct line:"<<s<<endl;
	// TODO: Add your control notification handler code here
	CString csDirPath;

	csDirPath+="edged images\\*.jpg";
	HANDLE file;
	WIN32_FIND_DATA fileData;
	//mbstowcs(fn,csDirPath.GetBuffer(),999);
	file = FindFirstFile(csDirPath.GetBuffer(), &fileData);
	cout<<fileData.cFileName<<endl;
	CString path = "edged images\\"+(CString)fileData.cFileName;
	detectLines(path);
	//m_FileList.push_back(fileData.cFileName);
	bool bState = false;
	bState = FindNextFile(file, &fileData);
	while(bState){
		//wcstombs(line,(const char*)fileData.cFileName,259);
		//m_FileList.push_back(fileData.cFileName);
		cout<<fileData.cFileName<<endl;
		path = "edged images\\"+(CString)fileData.cFileName;
		detectLines(path);
		bState = FindNextFile(file, &fileData);
	
	}
	cout<<"detection done"<<endl;
	return;

}

/*decomposition code*/
void doDecomposition(CString filename){
	string name = filename.GetBuffer(0);
	Mat src = imread(name, 1);
	Mat background = Mat::zeros(src.rows,src.cols,CV_32FC3);
	String path_hough="detected\\"+name+"_hough_showlines.jpg";
	if(src.empty())
	{
		cout << "can not open " << name << endl;
		return;
	}
	Mat dst, cdst;
	int thresh1=50,thresh2=200;
	char cc;

	Canny(src, dst, thresh1, thresh2, 3);
	//cvtColor(src,dst,CV_BGR2GRAY);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	vector<Vec4i> lines,verhorizon_lines,nolapping_lines,partion_lines;
	vector<Rec> rects;
	vector<Segment> segs,verhorizon_segs,nolapping_segs;
	//HoughLinesP(dst, lines, 1, CV_PI/180, 20, MIN_LENGTH, 20 );
	HoughLinesP(dst, lines, 1, CV_PI/180, 10, 10 , 20);
	Vec4i2Segments(lines,segs);

	removeNonVertHorizonSegments(segs,verhorizon_segs);

	removeOverlappingSegments(verhorizon_segs,nolapping_segs);

	//Vec4i2Segments(nolapping_lines,nolapping_segs);

	symmetricCompletion(src,nolapping_segs,rects);
	
	//removeOverlappingSegments(nolapping_lines,partion_lines);
	//segmentsTolines(reduced_lines,&partion_lines);
	//s.findPartitionSegment(reduced_lines,partion_lines);
	/*hough detection visualization part*/

	/*Segment s1,s2,s3,s4;
	 for(int i=0;i<rects.size();i++){
		 s1 = rects[i].s1;
		 s2 = rects[i].s2;
		 s3 = rects[i].s3;
		 s4 = rects[i].s4;

		 line( background, s1.startp, s1.endp, Scalar(0,255,0), 2, CV_AA);
		 line( background, s2.startp, s2.endp, Scalar(0,255,0), 2, CV_AA);
		 line( background, s3.startp, s3.endp, Scalar(0,255,0), 2, CV_AA);
		 line( background, s4.startp, s4.endp, Scalar(0,255,0), 2, CV_AA);
		 drawcross(background,rects[i].center,Scalar(0,0,255),2);
		 namedWindow("detectedRect",0);
		 imshow("detectedRect",background);
		 cv::waitKey(0);
		 background = src.clone();
	 }*/

	cout<<"reduced line number:"<<nolapping_segs.size()<<endl;
	for( size_t i = 0; i < nolapping_segs.size(); i++ )
	{
		Segment l = nolapping_segs[i];
		line( background, l.startp, l.endp, Scalar(0,255,0), 1, CV_AA);
		line(background,Point(background.cols/2,0),Point(background.cols/2,background.rows),Scalar(0,0,255), 1, CV_AA);
	}
	imwrite(path_hough,background);
	namedWindow("facade",0);
 	imshow("facade",background);
	cv::waitKey(0);
	/*cc=waitKey();
	if(cc == 'a') thresh1 += 10;
	if(cc == 'd') thresh1 -= 10;
	if(cc == 'q') thresh2 += 10;
	if(cc == 'e') thresh2 -= 10;
	cout<<"thresh1"<<thresh1<<"    thresh2"<<thresh2<<endl;*/


	return;
}

void CImageProcessingDlg::OnBnClickeddecomposition()
{
	// TODO: 在此添加控件通知处理程序代码
	CString csDirPath;
	csDirPath+="facade\\*.jpg";
	HANDLE file;
	WIN32_FIND_DATA fileData;
	//mbstowcs(fn,csDirPath.GetBuffer(),999);
	file = FindFirstFile(csDirPath.GetBuffer(), &fileData);
	cout<<fileData.cFileName<<endl;
	CString path = "facade\\"+(CString)fileData.cFileName;
	doDecomposition(path);
	//m_FileList.push_back(fileData.cFileName);
	bool bState = false;
	bState = FindNextFile(file, &fileData);
	while(bState){
		//wcstombs(line,(const char*)fileData.cFileName,259);
		//m_FileList.push_back(fileData.cFileName);
		cout<<fileData.cFileName<<endl;
		path = "facade\\"+(CString)fileData.cFileName;
		doDecomposition(path);
		bState = FindNextFile(file, &fileData);
	
	}
	cout<<"decomposition done"<<endl;
	return;
}

void CImageProcessingDlg::OnBnClickedequalizehist()
{
	/*exaggerate the RGB value of a image*/

	Mat src, dst;
	CString csDirPath;
	csDirPath+="facade element depth\\*.jpg";
	HANDLE file;
	WIN32_FIND_DATA fileData;
	//mbstowcs(fn,csDirPath.GetBuffer(),999);
	file = FindFirstFile(csDirPath.GetBuffer(), &fileData);
	cout<<fileData.cFileName<<endl;
	CString path = "facade element depth\\"+(CString)fileData.cFileName;
	string name = path.GetBuffer(0);

	string path_Hist="detected\\"+name;//存储图像

	/// 加载源图像
	src = imread( name, 1 );
	if( !src.data )
	{ 
		cout<<"Usage: ./Histogram_Demo <path_to_image>"<<endl;
		return;
	}
	/// 转为灰度图
	cvtColor( src, src, CV_BGR2GRAY );
	/// 应用直方图均衡化
	equalizeHist( src, dst );
	
	cvtColor( dst, dst, CV_GRAY2BGR );
	
	imwrite(path_Hist,dst);

	/*/// 显示结果
	namedWindow( "source_window", 0 );
	namedWindow( "equalized_window", 0 );

	imshow( "source_window", src );
	imshow( "equalized_window", dst );

	/// 等待用户按键退出程序
	waitKey(0);*/

	bool bState = false;
	bState = FindNextFile(file, &fileData);

	while(bState){
		//wcstombs(line,(const char*)fileData.cFileName,259);
		//m_FileList.push_back(fileData.cFileName);
		cout<<fileData.cFileName<<endl;

		path = "facade element depth\\"+(CString)fileData.cFileName;
		name = path.GetBuffer(0);
		path_Hist="detected\\"+name;

		src = imread( name, 1 );
		cvtColor( src, src, CV_BGR2GRAY );
		equalizeHist( src, dst );
		cvtColor( dst, dst, CV_GRAY2BGR );

		imwrite(path_Hist,dst);

		/*imshow( "source_window", src );
		imshow( "equalized_window", dst );
		waitKey(0);*/

		bState = FindNextFile(file, &fileData);
	}

	return;
}

void CImageProcessingDlg::OnBnClickedaveragedepth()
{
	// TODO: 在此添加控件通知处理程序代码
	// get the average micro-depth value of the facade element using mask.
	Mat src,mask,dst;
	src = imread("depth17.jpg",0);
	mask = imread("img17mask.jpg",1);
	dst = Mat::zeros(src.rows,src.cols,CV_32FC1);

	long ele1=0,ele2=0,ele3=0,ele4=0,ele5=0;
	int count1=0,count2=0,count3=0,count4=0,count5=0;
	for(int i=0;i<src.rows;i++)
		for(int j=0;j<src.cols;j++){
			uchar d = src.at<uchar>(i,j);
			Vec3b m = mask.at<Vec3b>(i,j);
			int mb = (int)m[0], mg = (int)m[1], mr = (int)m[2];
			
			if(mb < 5 && mg < 5 && mr < 5){
				count1++;
				ele1 += (int)d;
			}
			else if(mb > 250 && mg > 250 && mr > 250){
				count2++;
				ele2 += (int)d;
			}
			else if(mb < 5 && mg < 5 && mr > 250){
				count3++;
				ele3 += (int)d;
			}
			else if(mb > 250 && mg > 250 && mr < 5){
				count4++;
				ele4 += (int)d;
			}
			else{
				count5++;
				ele5 += (int)d;
			}
		}
		int avg1 = ele1 / count1;
		int avg2 = ele2 / count2;
		int avg3 = ele3 / count3;
		int avg4 = ele4 / count4;
		int avg5 = ele5 / count5;

	for(int i=0;i<src.rows;i++)
		for(int j=0;j<src.cols;j++){
			Vec3b m = mask.at<Vec3b>(i,j);
			int mb = (int)m[0], mg = (int)m[1], mr = (int)m[2];
			
			if(mb < 5 && mg < 5 && mr< 5){
				dst.at<float>(i,j) = avg1;
			}
			else if(mb > 250 && mg > 250 && mr > 250){
				dst.at<float>(i,j) = avg2;
			}
			else if(mb < 5 && mg < 5 && mr > 250){
				dst.at<float>(i,j) = avg3;
			}
			else if(mb > 250 && mg > 250 && mr < 5){
				dst.at<float>(i,j) = avg4;
			}
			else{
				dst.at<float>(i,j) = avg5;
			}
	}
		imwrite("dst.jpg",dst);
}
