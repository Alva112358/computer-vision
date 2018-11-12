/*******************************************************************************
* PROGRAM : Computer Vision HW2
* AUTHOR  : Liang Junhua
* DATE    : 2018/10/14
*******************************************************************************/
#ifndef _CANNYSOURCE_ 
#define _CANNYSOURCE_
#define M_PI 3.1415926535
#define NOEDGE 255
#define POSSIBLE_EDGE 128
#define BOOSTBLURFACTOR 90.0
#define EDGE 0


#include <iostream>
#include <cmath>
#include <string>
#include <cstring>
#include <vector>
#include <queue>
#include <stack>
#include <utility>
#include <sstream>
#include "CImg.h"


using namespace std;
using namespace cimg_library;


class MyCanny {
public:
    MyCanny(char* filename, float sigma, float tlow, float thigh, int len);
    bool read_pgm_image(char* filename);
    bool write_pgm_image(char* filename, float sigma, float tlow, float thigh, char name[5]);
    void gaussian_smooth(float sigma);
    void non_max_supp();
    void follow_edges(CImg<unsigned char>& edgemap, int r, int c, int lowval);
    void apply_hysteresis(float tlow, float thigh);
    void edge_linking(int len);
    double angle_radians(double x, double y);
    vector<float> createGaussKernel(float sigma);
    vector<vector<short int>> derrivative_x();
    vector<vector<short int>> derrivative_y();
    vector<vector<short int>> magnitude_x_y();
    vector<vector<float>> radian_direction(int xdirtag, int ydirtag);
private:
    //CImg<unsigned char> bmp_image;            /* The origin bmp image */
    CImg<unsigned char> pgm_image;              /* The grey pgm image */
    CImg<short int> gaussian_smooth_image;      /* The gaussian smooth result image */
    CImg<unsigned char> non_max_image;          /* non_max_suppress image */
    CImg<unsigned char> edge_image;             /* The image after hysteresis */
    CImg<unsigned char> edge_linking_image;     /* The edge linking */
    vector<vector<short int>> magnitude;        /* The magnitude of the derrivation */
    vector<vector<short int>> delta_x;          /* The derrivative of x-direction */
    vector<vector<short int>> delta_y;          /* The derrivative of y-direction */
    vector<vector<float>> radians;              /* The radians of derrivation */
    vector<float> gaussKernel;                  /* The gauss kernel */
    CImg<unsigned char> createGrayImage();      /* Create a grey image */
    float gaussFunction(float sigma, float x);  /* Gauss Function with standard deviation sigma. */
};


#endif //_CANNYSOURCE_