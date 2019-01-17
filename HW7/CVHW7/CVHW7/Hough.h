#pragma once

#include "CImg.h"
#include <vector>
#include <iostream>
#include <set>
#include <math.h>
#define HOUGH_ROWS 500 //Divide the angle into some pieces.
#define M_PI 3.1415926535

#define EDGE 0
#define NOEDGE 255


using namespace std;
using namespace cimg_library;

struct Line {
	int t;
	int r;
	Line(int _t, int _r) : t(_t), r(_r) {}
};


struct pek {
	int t;
	int r;
	int peek;
	pek(int _t, int _r, int _peek) : t(_t), r(_r), peek(_peek) {}
};


class Hough {
public:
	Hough(CImg<unsigned char> t_image, float thre, float dise);
	void hough_vote();
	void drawLine(float thre, float dise);
	void findRelativeEdge();
	vector<pek> findPeeks(float thre, float dise);
	vector<pair<int, int>> getIntersection();
private:
	int rows;
	int cols;
	int hough_rows;
	int hough_cols;
	int max_vote;
	int maxLength;
	double radian;
	CImg<unsigned char> image;
	CImg<unsigned char> edgeImage;
	CImg<int> Hough_space;
	vector<pek> votePeek;
	vector<vector<Line>> paperLine;
	vector<pair<int, int>> intersection;
	set<pair<int, int>> linePoint;
};