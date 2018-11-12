/*****************************************************************
* Program : Computer Vision HW4
* Author  : Liang Junhua
* Date    : 2018/11/4
******************************************************************/

#ifndef _EQUALIZATION_
#define _EQUALIZATION_
#define GREY_IMAGE 0
#define RGB_IMAGE 1
#define _USE_MATH_DEFINES
#define Exp 2.71828

#include "CImg.h"
#include <vector>

using namespace cimg_library;
using namespace std;

class Equalization {
public:
    Equalization(char* filename, int TAG);
    ~Equalization();
    void createGreyImage();
    void createHistogram();
    void createHistogramRGB();
private:
    int rows;
    int cols;
    CImg<int> RGBImage;
    CImg<int> greyImage;
    CImg<int> histogramImage;
};

class Convertion {
public:
    Convertion(char* sourceFile, char* targetFile);
    ~Convertion();
    void createLABImage(CImg<float>& source, CImg<float>& lab, vector<float>& mean, vector<float>& sigma); /* Create the source's and the target's Lab image. */
    void createFinalImage();
private:
    int rows;
    int cols;
    vector<float> sourceMean;
    vector<float> targetMean;
    vector<float> sourceSigma;
    vector<float> targetSigma;
    CImg<float> sourceImage;
    CImg<float> targetImage;
    CImg<float> sourceLABImage;
    CImg<float> targetLABImage;
    CImg<float> finalResult;
};

#endif //_EQUALIZATION_