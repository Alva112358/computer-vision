#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include "CImg.h"

using namespace std;
using namespace cimg_library;

class Segmentation {
public:
    Segmentation(string fileName);
    ~Segmentation();
    CImg<float> createGreyImage(CImg<float> _image);
    vector<int> createHistogram(CImg<float> greyImage);
    vector<float> createNormalizeHistogram(vector<int> histogram);
    int createBestThreshold(vector<float> probability, vector<int> histogram);
    CImg<float> segmentation(int threshold);
    void getDerrivative(CImg<float> result);
    void edgeDetect(CImg<float> result);
    CImg<unsigned char> getFinalResult();
private:
    CImg<float> image;
    CImg<unsigned char> finalResult;
    vector<vector<int>> detX;
    vector<vector<int>> detY;
};
