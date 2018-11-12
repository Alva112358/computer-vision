/*****************************************************************
* Program : Computer Vision HW4
* Author  : Liang Junhua
* Date    : 2018/11/4
******************************************************************/

#include "Equalization.h"
#include <iostream>
#include <math.h>


/* Constructor. */
Equalization::Equalization(char* filename, int TAG) {
    RGBImage.load_bmp(filename);
    rows = RGBImage._width;
    cols = RGBImage._height;
    RGBImage.display("RGBImage" ,false);

    /* Grey image */
    if (TAG == GREY_IMAGE) {
        createGreyImage();
        CImg<float> temp = greyImage;
        CImg<int> histImg = temp.histogram(256, 0, 255);
        histImg.display_graph("Histogram", 3);

        createHistogram();
        temp = histogramImage;
        histImg = temp.histogram(256, 0, 255);
        histImg.display_graph("Hist2", 3);

        greyImage.display("Grey Image", false);
        histogramImage.display("Histogram Image Of Grey Image", false);

        /* Save the image. */
        string File = filename;
        File = File.substr(0, File.length()-4);
        string greyFile = File;
        greyFile += "_grey.bmp";
        File += "_result_grey.bmp";
        greyImage.save(greyFile.c_str());
        histogramImage.save(File.c_str());
    }

    /* Colorful image. */
    if (TAG == RGB_IMAGE) {
        createHistogramRGB();
        histogramImage.display("Histogram Image Of Colorful Image", false);

        /* Save the image. */
        string File = filename;
        File = File.substr(0, File.length()-4);
        File += "_result_color.bmp";
        histogramImage.save(File.c_str());
    }
}


Equalization::~Equalization() {}


/* Create the gray image. */
void Equalization::createGreyImage() {
    greyImage = CImg<int>(rows, cols, 1, 1, 0);
    cimg_forXY(RGBImage, r, c) {
        float red = RGBImage(r, c, 0);
        float green = RGBImage(r, c, 1);
        float blue = RGBImage(r, c, 2);
        int value = red*0.299 + green*0.587 + blue*0.114;
        greyImage(r, c) = value;
    }
}


/* Create the histogram. */
void Equalization::createHistogram() {
    /* Grey image histogram. */
    histogramImage = CImg<int>(rows, cols, 1, 1, 0);
    int maxValue = 0;
    int totalGreyNumber = rows*cols;
    float grayScaleNumber[256] = {0};
    float probability[256] = {0};
    float cumulative[256] = {0};

    cimg_forXY(greyImage, r, c) {
        int greyNumber = greyImage(r, c);
        grayScaleNumber[greyNumber]++;
        maxValue = (maxValue > greyNumber) ? maxValue : greyNumber;
    }
    //cout << totalGreyNumber << endl;

    for (int i = 0; i < 256; i++) {
        probability[i] = grayScaleNumber[i] / totalGreyNumber;
        for (int j = 0; j <= i; j++) {
            cumulative[i] += probability[j]; 
        }
    }

    cimg_forXY(greyImage, r, c) {
        int greyIndex = greyImage(r, c);
        histogramImage(r, c) = (int)(cumulative[greyIndex] * maxValue);
    }
}


/* Create RGB Histogram. */
void Equalization::createHistogramRGB() {
    histogramImage = CImg<int>(rows, cols, 1, 3, 0);
    int totalNumber = rows*cols;
    int RMaxValue = 0;
    int BMaxValue = 0;
    int GMaxValue = 0;
    vector<vector<float>> RGBScaleNumber(3, vector<float>(256, 0.0));
    vector<vector<float>> RGBProbability(3, vector<float>(256, 0.0));
    vector<vector<float>> RGBCumulative(3, vector<float>(256, 0.0));

    cimg_forXY(RGBImage, r, c) {
        int rValue = RGBImage(r, c, 0);
        int gValue = RGBImage(r, c, 1);
        int bValue = RGBImage(r, c, 2);
        RGBScaleNumber[0][rValue]++;
        RGBScaleNumber[1][gValue]++;
        RGBScaleNumber[2][bValue]++;
        RMaxValue = (RMaxValue > rValue) ? RMaxValue : rValue;
        GMaxValue = (GMaxValue > gValue) ? GMaxValue : gValue;
        BMaxValue = (BMaxValue > bValue) ? BMaxValue : bValue;
    }

    for (int i = 0; i < 256; i++) {
        RGBProbability[0][i] = RGBScaleNumber[0][i] / totalNumber;
        RGBProbability[1][i] = RGBScaleNumber[1][i] / totalNumber;
        RGBProbability[2][i] = RGBScaleNumber[2][i] / totalNumber;
        for (int j = 0; j <= i; j++) {
            RGBCumulative[0][i] += RGBProbability[0][j];
            RGBCumulative[1][i] += RGBProbability[1][j];
            RGBCumulative[2][i] += RGBProbability[2][j];
        }
    }

    cimg_forXY(RGBImage, r, c) {
        int HIndex = RGBImage(r, c, 0);
        int SIndex = RGBImage(r, c, 1);
        int IIndex = RGBImage(r, c, 2);
        histogramImage(r, c, 0) = (RGBCumulative[0][HIndex] * RMaxValue);
        histogramImage(r, c, 1) = (RGBCumulative[1][SIndex] * GMaxValue);
        histogramImage(r, c, 2) = (RGBCumulative[2][IIndex] * BMaxValue);
    }
}


Convertion::Convertion(char* sourceFile, char* targetFile) {
    sourceImage = CImg<float>(sourceFile);
    targetImage = CImg<float>(targetFile);
    rows = sourceImage._width;
    cols = sourceImage._height;
    sourceMean = vector<float>(3, 0.0);
    targetMean = vector<float>(3, 0.0);
    sourceSigma = vector<float>(3, 0.0);
    targetSigma = vector<float>(3, 0.0);

    createLABImage(sourceImage, sourceLABImage, sourceMean, sourceSigma);
    createLABImage(targetImage, targetLABImage, targetMean, targetSigma);
    createFinalImage();

    sourceImage.display("sourceImage", false);
    targetImage.display("targetImage", false);
    //LABImage.display("LABImage", false);
    /*
    for (int i = 0; i < 3; i++) {
        printf("sourceMean:%d: %f\n", i, sourceMean[i]);
        printf("targetMean:%d: %f\n", i, targetMean[i]);
        printf("sourceVari:%d: %f\n", i, sourceSigma[i]);
        printf("targetVari:%d: %f\n", i, targetSigma[i]);
    }*/
    finalResult.display("Final", false);
    finalResult.normalize(0, 255);

    /* Save the image. */
    string File = sourceFile;
    File = File.substr(0, File.length()-4);
    File += "_result.bmp";
    finalResult.save(File.c_str());
}

Convertion::~Convertion() {}



void Convertion::createLABImage(CImg<float>& source, CImg<float>& lab, vector<float>& mean, vector<float>& sigma) {
    lab = CImg<float>(source._width, source._height, 1, 3, 0);
    float sum[3] = {0};
    float totalNumber = source._width * source._height;

    /* source image. */
    cimg_forXY(source, r, c) {
        float R = source(r, c, 0);
        float G = source(r, c, 1);
        float B = source(r, c, 2);

        /* From RGB to LMS. */
        float L = 0.3811*R + 0.5783*G + 0.0402*B;
        float M = 0.1967*R + 0.7244*G + 0.0782*B;
        float S = 0.0241*R + 0.1288*G + 0.8444*B;

        L = (L == 0) ? 0 : log(L);
        M = (M == 0) ? 0 : log(M);
        S = (S == 0) ? 0 : log(S);

        /* From LMS to Lab. */
        float L1 = (L + M + S);
        float M1 = (L + M - 2*S);
        float S1 = (L - M);

        float l = L1 / sqrt(3.0);
        float a = M1 / sqrt(6.0);
        float b = S1 / sqrt(2.0); 

        /* Accumulate weights. */
        sum[0] += l;
        sum[1] += a;
        sum[2] += b;

        lab(r, c, 0) = l;
        lab(r, c, 1) = a;
        lab(r, c, 2) = b;
    }

    mean[0] = sum[0] / totalNumber;
    mean[1] = sum[1] / totalNumber;
    mean[2] = sum[2] / totalNumber;

    cimg_forXY(lab, r, c) {
        sigma[0] += ((mean[0] - lab(r, c, 0)) * (mean[0] - lab(r, c, 0)));
        sigma[1] += ((mean[1] - lab(r, c, 1)) * (mean[1] - lab(r, c, 1)));
        sigma[2] += ((mean[2] - lab(r, c, 2)) * (mean[2] - lab(r, c, 2)));
    }

    sigma[0] /= totalNumber;
    sigma[1] /= totalNumber;
    sigma[2] /= totalNumber;

    sigma[0] = sqrt(sigma[0]);
    sigma[1] = sqrt(sigma[1]);
    sigma[2] = sqrt(sigma[2]);
}


void Convertion::createFinalImage() {
    finalResult = CImg<float>(rows, cols, 1, 3, 0);
    float sigmaRatio[3] = {0};
    sigmaRatio[0] = targetSigma[0] / sourceSigma[0];
    sigmaRatio[1] = targetSigma[1] / sourceSigma[1];
    sigmaRatio[2] = targetSigma[2] / sourceSigma[2];

    cimg_forXY(finalResult, r, c) {
        float L = sigmaRatio[0] * (sourceLABImage(r, c, 0) - sourceMean[0]) + targetMean[0];
        float A = sigmaRatio[1] * (sourceLABImage(r, c, 1) - sourceMean[1]) + targetMean[1];
        float B = sigmaRatio[2] * (sourceLABImage(r, c, 2) - sourceMean[2]) + targetMean[2];
        finalResult(r, c, 0) = L;
        finalResult(r, c, 1) = A;
        finalResult(r, c, 2) = B;
    }

    /* Lab to RGB. */
    cimg_forXY(finalResult, r, c) {
        float L = finalResult(r, c, 0) / sqrt(3.0) + finalResult(r, c, 1) / sqrt(6.0) + finalResult(r, c, 2) / sqrt(2.0);
        float M = finalResult(r, c, 0) / sqrt(3.0) + finalResult(r, c, 1) / sqrt(6.0) - finalResult(r, c, 2) / sqrt(2.0);
        float S = finalResult(r, c, 0) / sqrt(3.0) - 2 * finalResult(r, c, 1) / sqrt(6.0);

        L = pow(Exp, L);
        M = pow(Exp, M);
        S = pow(Exp, S);

        float R = 4.4679*L - 3.5873*M + 0.1193*S;
        float G = -1.2186*L + 2.3809*M - 0.1624*S;
        float B = 0.0497*L - 0.2439*M + 1.2045*S;

        finalResult(r, c, 0) = R;
        finalResult(r, c, 1) = G;
        finalResult(r, c, 2) = B;
    }
}


/* Main Test. */
int main(int argc, char* argv[]) {
    if (strcmp(argv[1], "Equalization") == 0) {
        printf("Equalization\n");
        Equalization equalization(argv[2], atoi(argv[3]));
    }
    else if (strcmp(argv[1], "Transfer") == 0) {
        printf("Transfer\n");
        Convertion convertion(argv[2], argv[3]);
    }
    return 0;
}