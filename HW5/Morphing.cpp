/*****************************************************************
* Program : Computer Vision HW5
* Author  : Liang Junhua
* Date    : 2018/11/9
******************************************************************/

#include "Morphing.h"
#include <iostream>
#include <sstream>

/* Constructor -- initializing all the basic parameters. */
Morphing::Morphing(char* sourceFile, char* targetFile, int _frame, 
        vector<point>& sourcePoint, 
        vector<point>& targetPoint,
        vector<vector<int>>& indexToPoint) {
    /* Initializing all the images and parameters. */
    printf("Initializing all the images and parameters...\n");
    sourceImage = CImg<float>(sourceFile);
    targetImage = CImg<float>(targetFile);
    frame = _frame;

    /* Initialize source and target triangle. */
    printf("Start initializing source and target triangle...\n");
    for (int i = 0; i < indexToPoint.size(); i++) {
        Triangle sourceTemp(sourcePoint[indexToPoint[i][0]], sourcePoint[indexToPoint[i][1]], sourcePoint[indexToPoint[i][2]]);
        Triangle targetTemp(targetPoint[indexToPoint[i][0]], targetPoint[indexToPoint[i][1]], targetPoint[indexToPoint[i][2]]);
        sourceTriangle.push_back(sourceTemp);
        targetTriangle.push_back(targetTemp);
    }

    /* Display the source image with lines. */
    unsigned char color[] = {255, 0, 0};
    CImg<float> lineImage = sourceImage;
    for (int j = 0; j < sourceTriangle.size(); j++) {
        lineImage.draw_line(sourceTriangle[j].a.first, sourceTriangle[j].a.second, sourceTriangle[j].b.first, sourceTriangle[j].b.second, color);
        lineImage.draw_line(sourceTriangle[j].a.first, sourceTriangle[j].a.second, sourceTriangle[j].c.first, sourceTriangle[j].c.second, color);
        lineImage.draw_line(sourceTriangle[j].b.first, sourceTriangle[j].b.second, sourceTriangle[j].c.first, sourceTriangle[j].c.second, color);
    }
    lineImage.display("Source", false);

    /* Display the target image with lines. */
    CImg<float> tineImage = targetImage;
    for (int j = 0; j < sourceTriangle.size(); j++) {
        tineImage.draw_line(targetTriangle[j].a.first, targetTriangle[j].a.second, targetTriangle[j].b.first, targetTriangle[j].b.second, color);
        tineImage.draw_line(targetTriangle[j].a.first, targetTriangle[j].a.second, targetTriangle[j].c.first, targetTriangle[j].c.second, color);
        tineImage.draw_line(targetTriangle[j].b.first, targetTriangle[j].b.second, targetTriangle[j].c.first, targetTriangle[j].c.second, color);
    }
    tineImage.display("target", false);

    vector<vector<point>> pointSet = createMiddlePointSet(sourcePoint, targetPoint);
    createMiddleTriangle(pointSet, indexToPoint);
    createFinalResult();

    //finalResult.save_video("Test");
    for (int i = 0; i < finalResult.size(); i++) {
        stringstream ss;
        ss << i+1;
        string temp;
        ss >> temp;
        string filename = "frame";
        filename += (temp+".bmp");
        finalResult[i].save(filename.c_str(), false);
    }
}


/* Deconstructor. */
Morphing::~Morphing() {
    printf("Thank you for using my morphing algorithm!\n");
}


/* Affine transform - computing the transform matrix. */
CImg<float> Morphing::createTransformMatrix(Triangle src, Triangle dst) {

    /* Use source triangle's vertex points to construct the computing matrix. */
    CImg<float> vertexMatrix(3, 3, 1, 1, 1);
    vertexMatrix(0, 0) = src.a.first; vertexMatrix(1, 0) = src.a.second; //1
    vertexMatrix(0, 1) = src.b.first; vertexMatrix(1, 1) = src.b.second; //1
    vertexMatrix(0, 2) = src.c.first; vertexMatrix(1, 2) = src.c.second; //1

    /* Use target triangle's x-coordinate and y-coordinate to construct the 1-st row and 2-rd row */
    CImg<float> target_x(1, 3, 1, 1, 0);
    CImg<float> target_y(1, 3, 1, 1, 0);
    target_x(0, 0) = dst.a.first;
    target_x(0, 1) = dst.b.first;
    target_x(0, 2) = dst.c.first;

    target_y(0, 0) = dst.a.second;
    target_y(0, 1) = dst.b.second;
    target_y(0, 2) = dst.c.second;

    /* Result rows is r1 and r2, solve the linear equations. */
    CImg<float> result_row_1 = target_x.solve(vertexMatrix);
    CImg<float> result_row_2 = target_y.solve(vertexMatrix);

    /* Finally, use r1 and r2 to construct the transform matrix. */
    CImg<float> transformMatrix(3, 3, 1, 1, 0);
    transformMatrix(0, 0) = result_row_1(0, 0);
    transformMatrix(1, 0) = result_row_1(0, 1);
    transformMatrix(2, 0) = result_row_1(0, 2);
    transformMatrix(0, 1) = result_row_2(0, 0);
    transformMatrix(1, 1) = result_row_2(0, 1);
    transformMatrix(2, 1) = result_row_2(0, 2);
    transformMatrix(2, 2) = 1;

    return transformMatrix;
}

/* Create the middle pointSet. */
vector<vector<point>> Morphing::createMiddlePointSet(vector<point>& sourcePoint, vector<point>& targetPoint) {
    /* Creating the middle point, source -> pointSet[i] -> target */
    vector<vector<point>> pointSet;
    for (int i = 0; i < frame; i++) {
        vector<point> temp;
        for (int j = 0; j < sourcePoint.size(); j++) {
            int mid_x = sourcePoint[j].first + (float)(i+1) / (frame+1) * (targetPoint[j].first - sourcePoint[j].first);
            int mid_y = sourcePoint[j].second + (float)(i+1) / (frame+1) * (targetPoint[j].second - sourcePoint[j].second);
            point mid_point(mid_x, mid_y);
            temp.push_back(mid_point);
        }
        pointSet.push_back(temp);
    }
    return pointSet;
}


/* Create the middle triangles. */
void Morphing::createMiddleTriangle(vector<vector<point>>& pointSet, vector<vector<int>>& indexToPoint) {
    printf("Start creating the middle Triangle...\n");

    /* Creating the middle Triangle. */
    for (int i = 0; i < frame; i++) {
        vector<Triangle> triangleTemp;
        for (int j = 0; j < indexToPoint.size(); j++) {
            Triangle temp(pointSet[i][indexToPoint[j][0]], pointSet[i][indexToPoint[j][1]], pointSet[i][indexToPoint[j][2]]);
            triangleTemp.push_back(temp);
        }
        middleTriangle.push_back(triangleTemp);
    }

    printf("Finish creating the middle Triangle\n");
}


/* Create the final result. */
void Morphing::createFinalResult() {
    printf("Start creating the final result...\n");
    /* Put the first frame into the result. */
    finalResult.push_back(sourceImage);
    CImg<float> middleFrame(targetImage._width, targetImage._height, 1, 3, 1);

    for (int k = 0; k < frame; k++) {
        cimg_forXY(middleFrame, x, y) {
            /* point in CImg format. */
            CImg<float> middleFramePoint(1, 3, 1, 1, 1);
            CImg<float> pointInSource(1, 3, 1, 1, 1);
            CImg<float> pointInTarget(1, 3, 1, 1, 1);
            /* For each point in middleFrame, check its triangle. */
            for (int m = 0; m < sourceTriangle.size(); m++) {
                point framePoint(x, y);
                /* If it's in this triangle. */ 
                if (middleTriangle[k][m].isInTriangle(framePoint)) {
                    middleFramePoint(0, 0) = x;
                    middleFramePoint(0, 1) = y;
                    /* Change from middle to source. */
                    CImg<float> transOne = createTransformMatrix(middleTriangle[k][m], sourceTriangle[m]);
                    /* Change from middle to target. */
                    CImg<float> transTwo = createTransformMatrix(middleTriangle[k][m], targetTriangle[m]);

                    //CImg<float> transMatrix = createTransformMatrix(sourceTriangle[m], targetTriangle[m]);
                    //Calculate the target point's location.
                    pointInSource = transOne * middleFramePoint;
                    pointInTarget = transTwo * middleFramePoint;

                    /* Interpolation. */
                    float ratio = float(k+1) / (frame+1);
                    middleFrame(x, y, 0) = (1 - ratio) * sourceImage(pointInSource(0, 0), pointInSource(0, 1), 0) + ratio * targetImage(pointInTarget(0, 0), pointInTarget(0, 1), 0);
                    middleFrame(x, y, 1) = (1 - ratio) * sourceImage(pointInSource(0, 0), pointInSource(0, 1), 1) + ratio * targetImage(pointInTarget(0, 0), pointInTarget(0, 1), 1);
                    middleFrame(x, y, 2) = (1 - ratio) * sourceImage(pointInSource(0, 0), pointInSource(0, 1), 2) + ratio * targetImage(pointInTarget(0, 0), pointInTarget(0, 1), 2);
                    break;
                }
            }
        }
        finalResult.push_back(middleFrame);
    }

    /* Put the last frame into the result. */
    finalResult.push_back(targetImage);
    printf("Finish creating the final result\n");
}


int main(int argc, char* argv[]) {
    int input;
    int x,y;
    int a,b,c;
    vector<point> sourcePoint;
    vector<point> targetPoint;
    vector<vector<int>> indexToPoint;

    for (int i = 0; i < 16; i++) {
        cin >> x >> y;
        point temp(x, y);
        sourcePoint.push_back(temp);
    }

    for (int i = 0; i < 16; i++) {
        cin >> x >> y;
        point temp(x, y);
        targetPoint.push_back(temp);
    }

    for (int i = 0; i < 20; i++) {
        cin >> a >> b >> c;
        vector<int> temp {a, b, c};
        indexToPoint.push_back(temp);
    }

    Morphing morphing(argv[1], argv[2], atoi(argv[3]), sourcePoint, targetPoint, indexToPoint);
    return 0;
}