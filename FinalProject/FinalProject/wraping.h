#pragma once
#include "CImg.h"
#include <utility>
#include <vector>

using namespace cimg_library;
using namespace std;

CImgList<float> createTransferMatrix(vector<pair<int, int>> intersection, CImg<float> image) {
	// printf("> Begin creating transform matrix\n");
    // Use two triangle to transfer.
    CImgList<float> TransferMatrix;

    // The A4 paper's four corners after warphing.
    vector<pair<int, int>> target;
    target.push_back(make_pair(0, 0));
    target.push_back(make_pair(image._width-1, 0));
    target.push_back(make_pair(0, image._height-1));
    target.push_back(make_pair(image._width-1, image._height-1));

    // Calculate the matrix.
    CImg<float> y1(1, 3, 1, 1, 0);
    CImg<float> y2(1, 3, 1, 1, 0);
    CImg<float> y3(1, 3, 1, 1, 0);
    CImg<float> y4(1, 3, 1, 1, 0);

    CImg<float> c1(1, 3, 1, 1, 0);
    CImg<float> c2(1, 3, 1, 1, 0);
    CImg<float> c3(1, 3, 1, 1, 0);
    CImg<float> c4(1, 3, 1, 1, 0);

    CImg<float> A1(3, 3, 1, 1, 1);
    CImg<float> A2(3, 3, 1, 1, 1);

    for (int i = 0; i < 3; i++) {
        A1(0, i) = target[i].first;
        A1(1, i) = target[i].second;
        A2(0, i) = target[3-i].first;
        A2(1, i) = target[3-i].second;

        y1(0, i) = intersection[i].first;
        y2(0, i) = intersection[i].second;
        y3(0, i) = intersection[3-i].first;
        y4(0, i) = intersection[3-i].second;
    }

    c1 = y1.solve(A1);
    c2 = y2.solve(A1);
    c3 = y3.solve(A2);
    c4 = y4.solve(A2);

    CImg<float> matrixOne(3, 3, 1, 1, 0);
    CImg<float> matrixTwo(3, 3, 1, 1, 0);
    for (int i = 0; i < 3; i++) {
        matrixOne(i, 0) = c1(0, i);
        matrixOne(i, 1) = c2(0, i);

        matrixTwo(i, 0) = c3(0, i);
        matrixTwo(i, 1) = c4(0, i);
    }

	matrixOne(0, 2) = 0;
	matrixOne(1, 2) = 0;
    matrixOne(2, 2) = 1;
	matrixTwo(0, 2) = 0;
	matrixTwo(1, 2) = 0;
    matrixTwo(2, 2) = 1;

    TransferMatrix.push_back(matrixOne);
    TransferMatrix.push_back(matrixTwo);

	// printf("> Complete creating transform matrix\n");
    return TransferMatrix;
}

CImg<float> Warphing(CImg<float> image, CImgList<float> transferMatrix) {
	// printf("> Begin warphing the A4 paper\n");
    CImg<float> result(image._width, image._height, 1, 3, 0);
    CImg<float> y(1, 2, 1, 1, 0);
    CImg<float> c(1, 2, 1, 1, 0);
    CImg<float> A(2, 2, 1, 1, 1);

    A(0, 0) = 0; 
    A(0, 1) = image._width-1;
    y(0, 0) = image._height-1; 
    y(0, 1) = 0;
    c = y.solve(A);

    CImg<float> srcPoint(1, 3, 1, 1, 1), dstPoint(1, 3, 1, 1, 1);
    cimg_forXY(image, x, y) {
        srcPoint(0, 0) = x;
        srcPoint(0, 1) = y;

        float dot = x * c(0, 0) - y + c(0, 1);
        if (dot >= 0)
            dstPoint = transferMatrix[0] * srcPoint;
        else
            dstPoint = transferMatrix[1] * srcPoint;

        if (dstPoint(0, 0) < 0) {
            dstPoint(0, 0) = 0;
        }
        else {
            if (dstPoint(0, 0) > image._width-1) {
                dstPoint(0, 0) = image._width-1;
            } else {
                dstPoint(0, 0) = dstPoint(0, 0);
            }
        }
            

        if (dstPoint(0, 1) < 0) {
            dstPoint(0, 1) = 0;
        }
        else {
            if (dstPoint(0, 1) > image._height-1) {
                dstPoint(0, 1) = image._height-1;
            } else {
                dstPoint(0, 1) = dstPoint(0, 1);
            }
        }

        for (int i = 0; i < 3; i++)
            result(x, y, i) = image(dstPoint(0, 0), dstPoint(0, 1), i);
    }

	// printf("> Warphing A4 paper finish!\n");
    return result;
}