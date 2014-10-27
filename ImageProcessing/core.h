/*core.h include the core functions and head files for geometry*/
#pragma once
#include "stdafx.h"
#include "BasicDataType.h"
#include<iostream>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>

#define MIN_DISTANCE 8 //min distance between 2 segments,the threshold to decide whether merge them or not
#define IMAGE_SCALE 1

using namespace std;
using namespace cv;

double distance1(const Point& p1,const Point& p2){
	return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) );
}

double _min(double a,double b){
	if(a>=b)
		return b;
	else 
		return a;
}

double _min(double d1,double d2,double d3,double d4){
	double min = d1;
	if(min > d2)
		min=d2;
	else if(min > d3)
		min = d3;
	else if(min>d4)
		min = d4;
	return min;
}

double _max(double a,double b){
	if(a>=b)
		return a;
	else 
		return b;
}

double _max(double d1,double d2,double d3,double d4){
	double max=d1;
	if(max < d2)
		max=d2;
	if(max < d3)
		max = d3;
	if(max < d4)
		max = d4;
	return max;
	
}

Mat getTsrMatrix(vector<Point>& points1,vector<Point>& points2){
	//估计两组点集之间的变换矩阵
	// 转换成解方程 A * X = B
	// X1 * S * cos - Y1 * sin + dx = X2;
	// X1 * sin + Y1 * S * cos + dy = Y2;

	if(points1.size() < 2)
	{
		cout << "get_tsr_matrix():not enough points." << endl;
		return Mat();
	}

	int rows = 2 * points1.size();
	Mat A = Mat::zeros( rows, 4, CV_64F );
	Mat X = Mat::zeros( 4, 1, CV_64F );
	Mat B = Mat::zeros( rows, 1, CV_64F );

	for( unsigned int i=0; i<points1.size(); i++)
	{
		A.at<double>(2*i, 0) = points1[i].x;
		A.at<double>(2*i, 1) = -points1[i].y;
		A.at<double>(2*i, 2) = 1;
		A.at<double>(2*i, 3) = 0;

		A.at<double>(2*i+1, 0) = points1[i].y;
		A.at<double>(2*i+1, 1) = points1[i].x;
		A.at<double>(2*i+1, 2) = 0;
		A.at<double>(2*i+1, 3) = 1;

		B.at<double>(2*i, 0) = points2[i].x;
		B.at<double>(2*i+1, 0) = points2[i].y;
	}

	solve( A, B, X, DECOMP_SVD ); 

	double s_cos = X.at<double>(0, 0);
	double sin = X.at<double>(1, 0);
	double dx = X.at<double>(2, 0);
	double dy = X.at<double>(3, 0);
	Mat tsr = (Mat_<double>(2, 3) << s_cos, -sin, dx, sin, s_cos, dy);

	return tsr;
}

Mat get_ls_value(vector<Point>&points){
	/*using least square method get line parameter a and b;
	min||Y-XA||^2
	W = (X'X)^-1 * X' * Y 
	X = [x1 1    W = [a;b]
		 x2 1
		 ....
		 xn 1]
	Y = [y1 y2 ... y3]'
	*/
	if(points.size()<2){
		cout<<"get_ls_value(): not enough points"<<endl;
		return Mat();
	}
	int n = points.size();
	Mat X = Mat::zeros(n,2,CV_64FC1);
	Mat Y = Mat::zeros(n,1,CV_64FC1);
	Mat W = Mat::zeros(2,1,CV_64FC1);
	for(int i=0;i<n;i++){
		X.at<double>(i,0) = points[i].x;
		X.at<double>(i,1) = 1;
		Y.at<double>(i,0) = points[i].y;
	}
	W = X.t()*X;
	W = W.inv();
	W = W*X.t()*Y;
	return W;
}

boolean notExist(int a,vector<int>& sets){
	for(int i=0;i<sets.size();i++)
		if(a == sets[i])
			return false;
	return true;
}

void Vec4i2Segments(vector<Vec4i>& vecs, vector<Segment>& segments){

	Segment s;
	for(int i=0;i<vecs.size();i++){
		s.startp = Point(vecs[i][0],vecs[i][1]);
		s.endp = Point(vecs[i][2],vecs[i][3]);
		segments.push_back(s);
	}
} 

void Segments2Vec4i(vector<Segment>& segments, vector<Vec4i>& vecs){
	Vec4i vec;
	for(int i=0;i<segments.size();i++){
		vec = Vec4i(segments[i].startp.x, segments[i].startp.y, segments[i].endp.x, segments[i].endp.y);
		vecs.push_back(vec);
	}
}

double distPoint2Segment(Point& p,Vec4i& v){
	/*distance between one point:p and a segment:v
2 cases:
1: point is ouside the segment: link point and a point of the segment as distance
2: point is in the middle of the segment: use line formation
（x1，y1) , (x2,y2) =>>   (x-x1)/(x2-x1)=(y-y1)/(y2-y1) =>> (y2-y1)(x-x1) = (x2-x1)(y-y1)
 =>> (y2-y1)x-(x2-x1)y+y1(x2-x1)-x1(y2-y1) = 0

*/
	double distance;
	Point p1(v[0],v[1]);
	Point p2(v[2],v[3]);

	Vec2i p1_p = Vec2i(p.x-p1.x,p.y-p1.y);
	Vec2i p1_p2 = Vec2i(p2.x-p1.x,p2.y-p1.y);

	Vec2i p2_p = Vec2i(p.x-p2.x,p.y-p2.y);
	Vec2i p2_p1 = Vec2i(p1.x-p2.x,p1.y-p2.y);

	if( (p1_p[0]*p1_p2[0]+p1_p[1]*p1_p2[1]) < 0){//point is outside p1 the segment
		distance = distance1(p,p1);
	}
	else if( (p2_p[0]*p2_p1[0]+p2_p[1]*p2_p1[1]) < 0){//point is outside p2 the segment
		distance = distance1(p,p2);
	}
	else{//point in the middle of the segment
		if(p2.x == p1.x){
			distance = abs(p.x - p1.x);
		}
		else if(p2.y == p1.y){
			distance = abs(p.y - p1.y);
		}
		else{
			double a = p2.y-p1.y;
			double b = p1.x-p2.x;
			double c = p1.y*(-b) - p1.x*a;
			distance = abs(a*p.x + b*p.y + c) / (sqrt(a*a+b*b));
		}
	}
	return distance;
		
}

double distPoint2Segment(Point& p,Segment& v){
	/*distance between one point:p and a segment:v
2 cases:
1: point is ouside the segment: link point and a point of the segment as distance
2: point is in the middle of the segment: use line formation
（x1，y1) , (x2,y2) =>>   (x-x1)/(x2-x1)=(y-y1)/(y2-y1) =>> (y2-y1)(x-x1) = (x2-x1)(y-y1)
 =>> (y2-y1)x-(x2-x1)y+y1(x2-x1)-x1(y2-y1) = 0

*/
	double distance;
	Point p1 = v.startp;
	Point p2 = v.endp;

	Vec2i p1_p = Vec2i(p.x-p1.x,p.y-p1.y);
	Vec2i p1_p2 = Vec2i(p2.x-p1.x,p2.y-p1.y);

	Vec2i p2_p = Vec2i(p.x-p2.x,p.y-p2.y);
	Vec2i p2_p1 = Vec2i(p1.x-p2.x,p1.y-p2.y);

	if( (p1_p[0]*p1_p2[0]+p1_p[1]*p1_p2[1]) < 0){//point is outside p1 the segment
		distance = distance1(p,p1);
	}
	else if( (p2_p[0]*p2_p1[0]+p2_p[1]*p2_p1[1]) < 0){//point is outside p2 the segment
		distance = distance1(p,p2);
	}
	else{//point in the middle of the segment
		if(p2.x == p1.x){
			distance = abs(p.x - p1.x);
		}
		else if(p2.y == p1.y){
			distance = abs(p.y - p1.y);
		}
		else{
			double a = p2.y-p1.y;
			double b = p1.x-p2.x;
			double c = p1.y*(-b) - p1.x*a;
			distance = abs(a*p.x + b*p.y + c) / (sqrt(a*a+b*b));
		}
	}
	return distance;
		
}
	
Vec2f distSegment2Segment(Vec4i& s1, Vec4i& s2){
	/*distance between 2 segments
  method:cacl each distance between a point and a segment
  using function distPoint2Segment() to calculate each 4 distance
  return the minmum distance
*/
	Point p11 = Point(s1[0],s1[1]);
	Point p12 = Point(s1[2],s1[3]);

	Point p21 = Point(s2[0],s2[1]);
	Point p22 = Point(s2[2],s2[3]);

	double distP11S2 = distPoint2Segment(p11,s2);
	double distP12S2 = distPoint2Segment(p12,s2);
	double distP21S1 = distPoint2Segment(p21,s1);
	double distP22S1 = distPoint2Segment(p22,s1);
	
	double min = distP11S2,max = distP11S2;
	if(min > distP12S2)
		min=distP12S2;
	if(min > distP21S1)
		min = distP21S1;
	if(min>distP22S1)
		min = distP22S1;

	if(max < distP12S2)
		max=distP12S2;
	if(max < distP21S1)
		max = distP21S1;
	if(max < distP22S1)
		max = distP22S1;

	return Vec2f(min,max);
}

Vec2f distSegment2Segment(Segment& s1, Segment& s2){
	/*distance between 2 segments
  method:cacl each distance between a point and a segment
  using function distPoint2Segment() to calculate each 4 distance
  return the minmum distance
*/
	Point p11 = s1.startp;
	Point p12 = s1.endp;

	Point p21 = s2.startp;
	Point p22 = s2.endp;

	double distP11S2 = distPoint2Segment(p11,s2);
	double distP12S2 = distPoint2Segment(p12,s2);
	double distP21S1 = distPoint2Segment(p21,s1);
	double distP22S1 = distPoint2Segment(p22,s1);
	
	double min = distP11S2,max = distP11S2;
	if(min > distP12S2)
		min=distP12S2;
	if(min > distP21S1)
		min = distP21S1;
	if(min>distP22S1)
		min = distP22S1;

	if(max < distP12S2)
		max=distP12S2;
	if(max < distP21S1)
		max = distP21S1;
	if(max < distP22S1)
		max = distP22S1;

	return Vec2f(min,max);
}

boolean equalsSegToSeg(Segment& s1,Segment& s2){
	if( (s1.startp == s2.startp && s1.endp == s2.endp) ||  (s1.startp == s2.endp && s1.endp == s2.startp) )
		return true;
	else
		return false;
}

boolean isSameLine(Line& a,Line& b){
	Point p11 = Point(a.point[0],a.point[1]);
	Point p12 = Point(a.point[2],a.point[3]);

	Point p21 = Point(b.point[0],b.point[1]);
	Point p22 = Point(b.point[2],b.point[3]);
	
	

	Point vec_a = Point( (p12.x-p11.x),(p12.y-p12.y) );
	Point vec_b = Point( (p22.x-p21.x),(p22.y-p21.y) );

	
	
	//余弦值30。 = 0.87  thresh>0.87最好接近于1
	double cos_theta = (vec_a.x * vec_b.x + vec_a.y * vec_b.y) / distance1(vec_a,vec_b);
	//
	////计算两条线段之间的最近距离 5pix
	double d1 = distance1(p11,p21);
	double d2 = distance1(p11,p22);
	double d3 = distance1(p12,p21);
	double d4 = distance1(p12,p22);
	

	double min = d1;
	if(min > d2)
		min=d2;
	else if(min > d3)
		min = d3;
	else if(min>d4)
		min = d4;
	if(cos_theta >= 0.87 && min <= MIN_DISTANCE * IMAGE_SCALE)
		return true;

	return true;//test'''
}

boolean isSameLine(Vec4i& a,Vec4i& b){
	Point p11 = Point(a[0],a[1]);
	Point p12 = Point(a[2],a[3]);

	Point p21 = Point(b[0],b[1]);
	Point p22 = Point(b[2],b[3]);

	Point vec_a = Point( (p12.x-p11.x),(p12.y-p11.y) );
	Point vec_b = Point( (p22.x-p21.x),(p22.y-p21.y) );

	//余弦值30。 = 0.87  thresh>0.87最好接近于1
	double cos_theta = (vec_a.x * vec_b.x + vec_a.y * vec_b.y) / (distance1(Point(0,0),vec_a) * distance1(Point(0,0),vec_b));
	//
	//计算两条线段之间的最近距离 5pix
	/*double d1 = distance1(p11,p21);
	double d2 = distance1(p11,p22);
	double d3 = distance1(p12,p21);
	double d4 = distance1(p12,p22);
	
	double min = d1;
	if(min > d2)
		min=d2;
	else if(min > d3)
		min = d3;
	else if(min>d4)
		min = d4;*/
	//Vec4i s1=a,s2=b;
	Vec2f min_max = distSegment2Segment(a,b);

	if(abs(cos_theta) >= 0.9 && min_max[0] <= MIN_DISTANCE * IMAGE_SCALE)
		return true;

	return false;//test'''
}

void drawcross(Mat& src,Point &pt, Scalar sclr, double width)
{
	Point up, down, left, right;
	up = down = left = right = pt;
	up.y = up.y - 10;
	down.y = down.y + 10;
	left.x=left.x-10;
	right.x=right.x+10;
	line(src, up, down, sclr, width);
	line(src, left, right, sclr, width);
}

Vec2i findNearestSeg(Point startp,Segment s1,Segment s2,Segment s3){
	/*findNearestSeg: find the nearest Segment and segment point from a start point
      return =Vec2i
	  Vec2i[0]: [0] [1] [2] represent for s1 s2 s3
	  Vec2i[1]: [0] [1] represent for start and end point
	*/
	
	Point p10 = s1.startp;
	Point p11  =s1.endp;

	Point p20 = s2.startp;
	Point p21 = s2.endp;

	Point p30 = s3.startp;
	Point p31 = s3.endp;

	double dstartp_p10 = distance1(startp,p10);
	double dstartp_p11 = distance1(startp,p11);
	double dstartp_p20 = distance1(startp,p20);
	double dstartp_p21 = distance1(startp,p21);
	double dstartp_p30 = distance1(startp,p30);
	double dstartp_p31 = distance1(startp,p31);

	if( (dstartp_p10 <= dstartp_p11) && (dstartp_p10 <= dstartp_p20) && (dstartp_p10 <= dstartp_p21) 
		&& (dstartp_p10 <= dstartp_p30) && (dstartp_p10 <= dstartp_p31) )
		return Vec2i(0,0);

	if( (dstartp_p11 <= dstartp_p10) && (dstartp_p11 <= dstartp_p20) && (dstartp_p11 <= dstartp_p21) 
		&& (dstartp_p11 < dstartp_p30) && (dstartp_p11 < dstartp_p31) )
		return Vec2i(0,1);

	if( (dstartp_p20 <= dstartp_p10) && (dstartp_p20 <= dstartp_p11) && (dstartp_p20 <= dstartp_p21) 
		&& (dstartp_p20 <= dstartp_p30) && (dstartp_p20 <= dstartp_p31) )
		return Vec2i(1,0);

	if( (dstartp_p21 <= dstartp_p10) && (dstartp_p21 <= dstartp_p11) && (dstartp_p21 <= dstartp_p20) 
		&& (dstartp_p21 <= dstartp_p30) && (dstartp_p21 <= dstartp_p31) )
		return Vec2i(1,1);

	if( (dstartp_p30 <= dstartp_p10) && (dstartp_p30 <= dstartp_p11) && (dstartp_p30 <= dstartp_p20) 
		&& (dstartp_p30 <= dstartp_p21) && (dstartp_p30 <= dstartp_p31) )
		return Vec2i(2,0);

	if( (dstartp_p31 <= dstartp_p10) && (dstartp_p31 <= dstartp_p11) && (dstartp_p31 <= dstartp_p20) 
		&& (dstartp_p31 <= dstartp_p21) && (dstartp_p31 <= dstartp_p30) )
		return Vec2i(2,1);
}

Vec2i findNearestSeg(Point startp,Segment s1,Segment s2){
	Point p10 = s1.startp;
	Point p11  =s1.endp;

	Point p20 = s2.startp;
	Point p21 = s2.endp;

	double dstartp_p10 = distance1(startp,p10);
	double dstartp_p11 = distance1(startp,p11);
	double dstartp_p20 = distance1(startp,p20);
	double dstartp_p21 = distance1(startp,p21);

	if( (dstartp_p10 <= dstartp_p11) && (dstartp_p10 <= dstartp_p20) && (dstartp_p10 <= dstartp_p21) )
		return Vec2i(0,0);

	if( (dstartp_p11 <= dstartp_p10) && (dstartp_p11 <= dstartp_p20) && (dstartp_p11 <= dstartp_p21) )
		return Vec2i(0,1);

	if( (dstartp_p20 <= dstartp_p10) && (dstartp_p20 <= dstartp_p11) && (dstartp_p20 <= dstartp_p21) )
		return Vec2i(1,0);

	if( (dstartp_p21 <= dstartp_p10) && (dstartp_p21 <= dstartp_p11) && (dstartp_p21 <= dstartp_p20) )
		return Vec2i(1,1);
}

Vec2i findNearestSeg(Point startp,Segment s1){
	Point p10 = s1.startp;
	Point p11  =s1.endp;

	double dstartp_p10 = distance1(startp,p10);
	double dstartp_p11 = distance1(startp,p11);

	if( (dstartp_p10 <= dstartp_p11) )
		return Vec2i(0,0);
	else
		return Vec2i(0,1);
}

Segment findSymmetrySegment(Segment& input,vector<Segment>& sets,int symmetry_axis,int thresh){
	for(int i=0;i<sets.size();i++){
		if(sets[i].startp.x == sets[i].endp.x){//vertical
			if( (distSegment2Segment(sets[i],Segment(Point(2*symmetry_axis-input.startp.x,input.startp.y),
													 Point(2*symmetry_axis-input.endp.x,input.endp.y)))[0]<thresh))
				/*&& abs(sets[i][0]-(2*symmetry_axis-input[0]) < thresh
				)*/
				return sets[i];
		}
		if(sets[i].startp.y == sets[i].endp.y){//horizontal
			if( (distSegment2Segment(sets[i],Segment(Point(2*symmetry_axis-input.startp.x,input.startp.y),
													 Point(2*symmetry_axis-input.endp.x,input.endp.y)))[0]<thresh))
				/*&& abs(sets[i][1]-input[1] < thresh)
				)*/
				return sets[i];
		}
	}
	return Segment(Point(0,0),Point(0,0));
}

Point findIntersectionSeg2Seg(Segment& s1,Segment& s2){
	/*find the intersection point of 2 segments using a thresh
		if there is no return point(0,0)*/
	int thresh = 20;
	if(s1.startp.x == s1.endp.x)//vertical
		if(s2.startp.y == s2.endp.y){
			if( ((s1.startp.y <= s2.endp.y && s1.endp.y >= s2.endp.y)||(s1.endp.y <= s2.endp.y && s1.startp.y >= s2.endp.y)) &&
				((s2.startp.x <= s1.endp.x && s2.endp.x >= s1.endp.x)||(s2.endp.x <= s1.endp.x && s2.startp.x >= s1.endp.x))
				){
					return Point(s1.startp.x,s2.startp.y);	
			}
			else{
				Vec2f d = distSegment2Segment(Vec4i(s1.startp.x,s1.startp.y,s1.endp.x,s1.endp.y),Vec4i(s2.startp.x,s2.startp.y,s2.endp.x,s2.endp.y));
				if(d(0) < thresh)
					return Point(s1.startp.x,s2.startp.y);
			}
		}
	if(s1.startp.y == s1.endp.y)//horizontal
		if(s2.startp.x == s2.endp.x){
			if( ((s2.startp.y <= s1.endp.y && s2.endp.y >= s1.endp.y)||(s2.endp.y <= s1.endp.y && s2.startp.y >= s1.endp.y)) &&
				((s1.startp.x <= s2.endp.x && s1.endp.x >= s2.endp.x)||(s1.endp.x <= s2.endp.x && s1.startp.x >= s2.endp.x))
				){
					return Point(s2.startp.x,s1.startp.y);	
				}
			else{
				Vec2f d = distSegment2Segment(Vec4i(s1.startp.x,s1.startp.y,s1.endp.x,s1.endp.y),Vec4i(s2.startp.x,s2.startp.y,s2.endp.x,s2.endp.y));
				if(d(0) < thresh)
					return Point(s2.startp.x,s1.startp.y);
			}
	}

	return Point(0,0);
}

Segment calcSymmetrySegment(Segment& input,int symmetry_axis){
	return Segment( Point(2*symmetry_axis-input.startp.x,input.startp.y),Point(2*symmetry_axis-input.endp.x,input.endp.y));
}

Segment clc_ORset_SymmetrySegment(Segment& input,Segment& symSeg,int symmetry_axis){
	if(symSeg.startp.x == symSeg.endp.x)//vertical
		return Segment( Point(symSeg.startp.x,_min(input.startp.y,input.endp.y,symSeg.startp.y,symSeg.endp.y)),
						Point(symSeg.endp.x,_max(input.startp.y,input.endp.y,symSeg.startp.y,symSeg.endp.y)));
	if(symSeg.startp.y == symSeg.endp.y)//horizontal
		return Segment( Point(_min(2*symmetry_axis-input.startp.x,2*symmetry_axis-input.endp.x,symSeg.startp.x,symSeg.endp.x),symSeg.startp.y),
						Point(_max(2*symmetry_axis-input.startp.x,2*symmetry_axis-input.endp.x,symSeg.startp.x,symSeg.endp.x),symSeg.endp.y));
}

double calcAngleOf2Vec(Point& vec1,Point& vec2){
	double cos_theta = (vec1.x * vec2.x + vec1.y * vec2.y) / (distance1(Point(0,0),vec1) * distance1(Point(0,0),vec2));
	return cos_theta;
}

void reorganize(vector<Segment>* segments){
	/*segments size==4 for rectamgle derection
	the result should be organized as a sequence that 4 segments are linked */
	//vector<Point>* tempPoints = new vector<Point>(*points);
	vector<Segment> tmpSegs = *segments;
	
	Segment startSeg = tmpSegs[0];
	Point startPoint = startSeg.startp;

	(*segments)[0] = startSeg;

	Vec2i v= findNearestSeg(startPoint,tmpSegs[1],tmpSegs[2],tmpSegs[3]);
	if(v[0] == 0){//tmp[1]
		(*segments)[1] = tmpSegs[1];
		if(v[1] == 0){//tmp[1] startpoint
			v = findNearestSeg(tmpSegs[1].endp,tmpSegs[2],tmpSegs[3]);
			if(v[0] == 0){
				(*segments)[2] = tmpSegs[2];
				(*segments)[3] = tmpSegs[3];
			}
			else{
				(*segments)[2] = tmpSegs[3];
				(*segments)[3] = tmpSegs[2];
			}
		}	
		else{
			v = findNearestSeg(tmpSegs[1].startp,tmpSegs[2],tmpSegs[3]);
			if(v[0] == 0){
				(*segments)[2] = tmpSegs[2];
				(*segments)[3] = tmpSegs[3];
			}
			else{
				(*segments)[2] = tmpSegs[3];
				(*segments)[3] = tmpSegs[2];
			}
		}
	}

		if(v[0] == 1){//tmp[2]
		(*segments)[1] = tmpSegs[2];
		if(v[1] == 0){//tmp[2] startpoint
			v = findNearestSeg(tmpSegs[2].endp,tmpSegs[1],tmpSegs[3]);
			if(v[0] == 0){
				(*segments)[2] = tmpSegs[1];
				(*segments)[3] = tmpSegs[3];
			}
			else{
				(*segments)[2] = tmpSegs[3];
				(*segments)[3] = tmpSegs[1];
			}
		}	
		else{
			v = findNearestSeg(tmpSegs[2].startp,tmpSegs[1],tmpSegs[3]);
			if(v[0] == 0){
				(*segments)[2] = tmpSegs[1];
				(*segments)[3] = tmpSegs[3];
			}
			else{
				(*segments)[2] = tmpSegs[3];
				(*segments)[3] = tmpSegs[1];
			}
		}
	}

		if(v[0] == 2){//tmp[3]
		(*segments)[1] = tmpSegs[3];
		if(v[1] == 0){//tmp[3] startpoint
			v = findNearestSeg(tmpSegs[3].endp,tmpSegs[1],tmpSegs[2]);
			if(v[0] == 0){
				(*segments)[2] = tmpSegs[1];
				(*segments)[3] = tmpSegs[2];
			}
			else{
				(*segments)[2] = tmpSegs[2];
				(*segments)[3] = tmpSegs[1];
			}
		}	
		else{
			v = findNearestSeg(tmpSegs[3].startp,tmpSegs[1],tmpSegs[2]);
			if(v[0] == 0){
				(*segments)[2] = tmpSegs[1];
				(*segments)[3] = tmpSegs[2];
			}
			else{
				(*segments)[2] = tmpSegs[2];
				(*segments)[3] = tmpSegs[1];
			}
		}
	}
	
	return;
}

Point calcCenterPoint(vector<Point>& points){
	int sum_x = 0,sum_y = 0;
	for(int i=0;i<points.size();i++){
		sum_x += points[i].x;
		sum_y += points[i].y;
	}
	return Point(sum_x/points.size(),sum_y/points.size());
}

boolean isSameRec(Rec& rec1, Rec& rec2){
	Segment s11 = rec1.s1;
	Segment s12 = rec1.s2;
	Segment s13 = rec1.s3;
	Segment s14 = rec1.s4;

	Segment s21 = rec2.s1;
	Segment s22 = rec2.s2;
	Segment s23 = rec2.s3;
	Segment s24 = rec2.s4;

	if(equalsSegToSeg(s11,s21) && equalsSegToSeg(s12,s22) && equalsSegToSeg(s13,s23) && equalsSegToSeg(s14,s24) )
		return true;
	return false;
}

boolean notExistSameLine(Segment& s1,Segment& s2,Segment& s3,Segment& s4){
	if(equalsSegToSeg(s1,s2) || equalsSegToSeg(s1,s3) || equalsSegToSeg(s1,s4) || equalsSegToSeg(s2,s3) || 
		equalsSegToSeg(s2,s4) || equalsSegToSeg(s3,s4) )
		return false;
	else
		return true;
}

boolean existSameRect(Rec& rect,vector<Rec>& rects){
	for(int i=0;i<rects.size();i++){
		if(isSameRec(rect,rects[i]))
			return true;
	}
	return false;
}
