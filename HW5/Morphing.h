/*****************************************************************
* Program : Computer Vision HW5
* Author  : Liang Junhua
* Date    : 2018/11/9
******************************************************************/

#ifndef _MORPHING_
#define _MORPHING_

#include "CImg.h"
#include <vector>
#include <utility>
#include <algorithm>

using namespace std;
using namespace cimg_library;
typedef pair<int, int> point;

struct Triangle {
    point a;
    point b;
    point c;
    Triangle(point _a, point _b, point _c) : a(_a), b(_b), c(_c) {}
    bool operator==(const Triangle& other) {
        return a == other.a && b == other.b && c == other.c;
    }
    bool isInTriangle(point& P) {
        point AB(b.first-a.first, b.second-a.second);
        point AC(c.first-a.first, c.second-a.second);
        point AP(P.first-a.first, P.second-a.second);
        float u = (dot(AB,AB)*dot(AP,AC)-dot(AC,AB)*dot(AP,AB)) / (dot(AC,AC)*dot(AB,AB)-dot(AC,AB)*dot(AB,AC));
        float v = (dot(AC,AC)*dot(AP,AB)-dot(AC,AB)*dot(AP,AC)) / (dot(AC,AC)*dot(AB,AB)-dot(AC,AB)*dot(AB,AC));
        //printf("u=%f v=%f u+v=%f\n", u, v, u+v);
        if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && u+v <= 1) return true;
        return false;
    }
    /*
    int crossProduct(point& p1, point& p2) {
        return p1.first * p2.second - p2.first * p1.second;
    }*/
    float dot(point& p1, point& p2) {
        float x1 = p1.first;
        float y1 = p1.second;
        float x2 = p2.first;
        float y2 = p2.second;
        return x1*x2+y1*y2;
    }
};

class Morphing {
public:
    Morphing(char* sourceFile, char* targetFile, int _frame, 
        vector<point>& sourcePoint, 
        vector<point>& targetPoint,
        vector<vector<int>>& indexToPoint);
    ~Morphing();
    CImg<float> createTransformMatrix(Triangle src, Triangle dst);
    vector<vector<point>> createMiddlePointSet(vector<point>& sourcePoint, vector<point>& targetPoint);
    void createMiddleTriangle(vector<vector<point>>& pointSet, vector<vector<int>>& indexToPoint);
    void createFinalResult();
private:
    int frame;
    CImg<float> sourceImage;
    CImg<float> targetImage;
    CImgList<float> finalResult;
    vector<Triangle> sourceTriangle;
    vector<Triangle> targetTriangle;
    vector<vector<Triangle>> middleTriangle;
};

#endif //_MORPHING_