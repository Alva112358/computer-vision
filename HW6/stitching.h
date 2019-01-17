/*****************************************************************
* Program : Computer Vision HW6
* Author  : Liang Junhua
* Date    : 2018/11/24
******************************************************************/
#ifndef _SITICHING_
#define _STITCHING_

#define _USE_MATH_DEFINES
#define NUM 4
#define RESIZE_SIZE 500.0
#define RANSAC_THRESHOLD 4.0
#define NUM_OF_PAIR 4

#include "CImg.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include "Eigen/Dense"
#include <set>
#include <queue>

using namespace std;
using namespace cimg_library;
using namespace Eigen;

extern "C" {
#include "vl/generic.h"
#include "vl/sift.h"
#include "vl/kdtree.h"
}

struct Point {
    VlSiftKeypoint a;
    VlSiftKeypoint b;
    Point(VlSiftKeypoint _x, VlSiftKeypoint _y) : a(_x), b(_y) {}
};

class Panorama {
public:
    Panorama(string filename);
    ~Panorama();
    void createGreyImage();


    /* 预处理 进行柱状转换 */
    void createCylinderImage();

    /* SIFT 提取关键点 */
    void createExtractFeature();
    vector<Point> getKeyPointMatch(map<vector<float>, VlSiftKeypoint>& feature_a,
     map<vector<float>, VlSiftKeypoint>& feature_b);

    /* RANSAC 过程 */
    int random(int min, int max);
    MatrixXf createHomography(vector<Point>& pairs);
    vector<int> createInlinerDataSet(vector<Point>& pairs, MatrixXf& H, set<int>& indices);
    MatrixXf leastSquareSolution(vector<Point>& pairs, vector<int>& idxs);
    vector<int> RANSAC(vector<Point>& pairs);

    /* Stitching 过程 */
    void replacePairs(vector<Point>& lhs, vector<Point>& rhs);
    vector<int> getAvgOffset(const vector<Point>& pairs, vector<int>& idxs);
    CImg<float> stitching();
    CImg<float> blendTwoImages(const CImg<float>& a, const CImg<float>& b, int offset_x, int offset_y, int min_x, int min_y);
private:
    int width;
    int height;
    CImgList<float> image;
    CImgList<float> grey_image;
    CImgList<float> cylinder_image;
    CImg<float> final_result;
    vector< map<vector<float>, VlSiftKeypoint> > features;
};


#endif //_SITICHING_
