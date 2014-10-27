#pragma once
#include "BasicDataType.h"
#include "core.h"

#define	THRESH_RECT_LENGTH  25 //min threshold that 2 segments can be 2 edges of a rectangle 
#define THRESH_RECT_VERTICAL 0.05   //min threshold that 2 segments can be 2 adjacent edges of a rectangle 
#define THRESH_RECT_PARALLEL 0.95 
#define THRESG_RECT_CENTER 0 //threshold for whether a point at(x,y) is the center for a rectangle
#define MIN_RECT_EDGELENGTH 10 //min radius
#define MAX_RECT_EDGELENGTH 250


//static void removeNonVertHorizonSegments(cv::vector<cv::Vec4i>& lines, cv::vector<cv::Vec4i>& reduced);
static cv::vector<Segment> decomposeSegments(Segment& s1,Segment& s2,Segment& s3,Segment& s4);

static void findPartitionSegment(cv::vector<cv::Vec4i>& lines, cv::vector<cv::Vec4i>& partions);

//static Rec findRectangle(Segment& s1,Segment& s2,Segment& s3,Segment& s4);
//static boolean isRectangle(Segment& line1,Segment& line2,Segment& line3,Segment& line4);

boolean isRectangle(Segment& line1,Segment& line2,Segment& line3,Segment& line4){
	/*analyze if the 4 input lines can form a rectangle
	  don not need to be accurate*/
	/*Point p11 = Point(line1[0],line1[1]);
	Point p12 = Point(line1[2],line1[3]);

	Point p21 = Point(line2[0],line2[1]);
	Point p22 = Point(line2[2],line2[3]);

	Point p31 = Point(line3[0],line3[1]);
	Point p32 = Point(line3[2],line3[3]);

	Point p41 = Point(line4[0],line4[1]);
	Point p42 = Point(line4[2],line4[3]);*/
	/*double slope1 = (p12.y-p11.y)/(p12.x-p11.x);
	double slope2 = (p22.y-p21.y)/(p22.x-p21.x);
	double slope3 = (p32.y-p31.y)/(p32.x-p31.x);
	double slope4 = (p42.y-p41.y)/(p42.x-p41.x);*/
	//double t1 = clock();

	int thresh = 20;//the threshhold for 2 adjacent segments

	if( !notExistSameLine(line1,line2,line3,line4)){
		//cout<<"exist same lines"<<endl
		return false;
	}

	vector<Segment> segments;
	//Segment s1,s2,s3,s4;
	/*s1.startp = p11;s1.endp = p12;
	s2.startp = p21;s2.endp = p22;
	s3.startp = p31;s3.endp = p32;
	s4.startp = p41;s4.endp = p42;*/



	segments.push_back(line1);
	segments.push_back(line2);
	segments.push_back(line3);
	segments.push_back(line4);

	reorganize(&segments);

	//for(int i=0;i<segments.size()-1;i++){
	//	Vec2f dis = distSegment2Segment(Vec4i(segments[i].startp.x,segments[i].startp.y,segments[i].endp.x,segments[i].endp.y),
	//							Vec4i(segments[(i+1)%segments.size()].startp.x,segments[(i+1)%segments.size()].startp.y,segments[(i+1)%segments.size()].endp.x,segments[(i+1)%segments.size()].endp.y) );

	//	if(dis[0] > thresh) return false;//dis[0]:minmum distance
	//}
		
			   
		

	/*segments[0]<=>segments[2]  segments[1]<=>segments[3]*/

	double length0 = distance1(segments[0].startp,segments[0].endp);
	double length1 = distance1(segments[1].startp,segments[1].endp);
	double length2 = distance1(segments[2].startp,segments[2].endp);
	double length3 = distance1(segments[3].startp,segments[3].endp);

	/*double slope0 = (segments[0].endp.y-segments[0].startp.y) / (segments[0].endp.x-segments[0].startp.x);
	double slope1 = (segments[1].endp.y-segments[1].startp.y) / (segments[1].endp.x-segments[1].startp.x);
	double slope2 = (segments[2].endp.y-segments[2].startp.y) / (segments[2].endp.x-segments[2].startp.x);
	double slope3 = (segments[3].endp.y-segments[3].startp.y) / (segments[3].endp.x-segments[3].startp.x);*/

	Point vec0 = Point(segments[0].endp.y-segments[0].startp.y,segments[0].endp.x-segments[0].startp.x);
	Point vec1 = Point(segments[1].endp.y-segments[1].startp.y,segments[1].endp.x-segments[1].startp.x);
	Point vec2 = Point(segments[2].endp.y-segments[2].startp.y,segments[2].endp.x-segments[2].startp.x);
	Point vec3 = Point(segments[3].endp.y-segments[3].startp.y,segments[3].endp.x-segments[3].startp.x);
	
	double cos_thetav0v1 = calcAngleOf2Vec(vec0,vec1);
	double cos_thetav1v2 = calcAngleOf2Vec(vec1,vec2);
	double cos_thetav2v3 = calcAngleOf2Vec(vec2,vec3);
	double cos_thetav3v0 = calcAngleOf2Vec(vec3,vec0);
	//double t2 = clock();
	//cout<<"isrectangle:"<<(t2-t1)/CLOCKS_PER_SEC<<"sec"<<endl;
	if( abs(length0-length2) <= THRESH_RECT_LENGTH * IMAGE_SCALE && abs(length1-length3) <= THRESH_RECT_LENGTH * IMAGE_SCALE && 
		abs(cos_thetav0v1) <= THRESH_RECT_VERTICAL * IMAGE_SCALE && abs(cos_thetav1v2) <= THRESH_RECT_VERTICAL * IMAGE_SCALE &&
		abs(cos_thetav2v3) <= THRESH_RECT_VERTICAL * IMAGE_SCALE && abs(cos_thetav3v0) <= THRESH_RECT_VERTICAL * IMAGE_SCALE )
		return true;
	
	return false;
}

vector<Segment> getRectSegments(Segment& line1,Segment& line2,Segment& line3,Segment& line4){
	/*inputs: 4 lines can be organized as a rectangle
	  output: a vector include 4 points that exactly form a rectangle*/
	vector<Segment> s;

	double length0 = distance1(line1.startp,line1.endp);
	double length1 = distance1(line2.startp,line2.endp);
	double length2 = distance1(line3.startp,line3.endp);
	double length3 = distance1(line4.startp,line4.endp);

	Point vec0 = Point(line1.endp.x-line1.startp.x,line1.endp.y-line1.startp.y);
	Point vec1 = Point(line2.endp.x-line2.startp.x,line2.endp.y-line2.startp.y);
	Point vec2 = Point(line3.endp.x-line3.startp.x,line3.endp.y-line3.startp.y);
	Point vec3 = Point(line4.endp.x-line4.startp.x,line4.endp.y-line4.startp.y);
	
	double cos_thetav0v1 = calcAngleOf2Vec(vec0,vec1);
	double cos_thetav1v2 = calcAngleOf2Vec(vec1,vec2);
	double cos_thetav2v3 = calcAngleOf2Vec(vec2,vec3);
	double cos_thetav3v0 = calcAngleOf2Vec(vec3,vec0);

	if(abs(cos_thetav0v1) >= THRESH_RECT_PARALLEL){
		s.push_back(line1);
		s.push_back(line3);
		s.push_back(line2);
		s.push_back(line4);
	}
	else if(abs(cos_thetav1v2) >= THRESH_RECT_PARALLEL){
		s.push_back(line2);
		s.push_back(line1);
		s.push_back(line3);
		s.push_back(line4);
	}
	else if(abs(cos_thetav2v3) >= THRESH_RECT_PARALLEL){
		s.push_back(line1);
		s.push_back(line3);
		s.push_back(line2);
		s.push_back(line4);
	}
	else if(abs(cos_thetav3v0) >= THRESH_RECT_PARALLEL){
		s.push_back(line1);
		s.push_back(line2);
		s.push_back(line4);
		s.push_back(line3);
	}
	else{//already organized well
		s.push_back(line1);
		s.push_back(line2);
		s.push_back(line3);
		s.push_back(line4);
	}
	if(s.size() != 4)
		cout<<"error"<<endl;
	return s;
}

Rec findRectangle(Segment& s1,Segment& s2,Segment& s3,Segment& s4){
	/*find if there is a rectangle from segment s1,s2,s3,s4
	  if there is no rect, return a rectangle and it's centerPoint =(0,0);
	*/
	vector<Segment>segments;
	Rec rect;
	segments.push_back(s1);
	segments.push_back(s2);
	segments.push_back(s3);
	segments.push_back(s4);

	reorganize(&segments);

	Point p01 = findIntersectionSeg2Seg(segments[0],segments[1]);
	Point p12 = findIntersectionSeg2Seg(segments[1],segments[2]);
	Point p23 = findIntersectionSeg2Seg(segments[2],segments[3]);
	Point p30 = findIntersectionSeg2Seg(segments[3],segments[0]);

	if( (p01.x == 0 && p01.y == 0) || (p12.x == 0 && p12.y == 0) || (p23.x == 0 && p23.y == 0) || (p30.x == 0 && p30.y == 0))//2 segments havo no intersection point
	{
		rect.center = Point(0,0);
		return rect;
	}
	
	Segment s11,s22,s33,s44;
	vector<Point> points;
	s11.startp = p01;s11.endp = p12;
	s22.startp = p12;s22.endp = p23;
	s33.startp = p23;s33.endp = p30;
	s44.startp = p30;s44.endp = p01;

	points.push_back(p01);points.push_back(p12);
	points.push_back(p23);points.push_back(p30);

	if(isRectangle(s11,s22,s33,s44)){
		Point center = calcCenterPoint(points);
		rect.center = center;
		rect.s1 = s11;rect.s2 = s22;rect.s3 = s33;rect.s4 = s44;
		return rect;
	}

}



