#pragma once
#include "CImg.h"
#include <queue>
#include <stack>
#include <vector>
#include <utility>
#include <list>
#include <fstream>
#include <string>
#include <cstdlib>

using namespace cimg_library;
using namespace std;

#define Binary_threshold 150
#define Binary_bound 30
#define NOEDGE 255
#define EDGE 0

class Process {
public:
	Process(CImg<float> _source, int threshold, int S_value, int connect_value);
	void createGreyImage();
	void createBinaryImage();
	void createBinaryImageByBernson(int size, int S, int Th);
	void deleteExtraPoint(int len, bool less);
	void findDividingLine();
	void divideIntoBarItemImg();
	vector<int> getSubImageX(CImg<float>& subImg);
	vector<int> get_inflection_points_inX(const CImg<float>& XHistogramImage);
	vector<CImg<float>> getRowImg(CImg<float>& lineImg, vector<int> divideX);
	void dilateImg(int barItemIndex);
	int dilate_process_xy(const CImg<float>& Img, int x, int y);
	int dilate_process_xxy(const CImg<float>& Img, int x, int y);
	void EightNeibourghLink(int barItemIndex);
	void saveSingleNumImg(string filename);
	void image_process(string filename);
private:
	int rows;
	int cols;
	int threshold;
	int connect_value;
	vector<pair<int, int>> dividePoints;
	vector<CImg<float>> subImageSet;
	vector<int> rowSize;
	CImg<float> source;
	CImg<float> greyImage;
	CImg<float> binaryImage;
	CImg<int> HistogramImage;
	CImg<int> DividingImg;
};