/*******************************************************************************
* PROGRAM : Computer Vision HW3 - Detect circles
* AUTHOR  : Liang Junhua
* DATE    : 2018/10/21
*******************************************************************************/

#ifndef _HOUGHCIRCLE_
#define _HOUGHCIRCLE_
#include "CImg.h"
#include "canny_source.h"

#define EDGE 0
#define NOEDGE 255
#define ANGLE_STEP 1  //Divide the angle into some pieces.
#define RADIUS_STEP 5

/* Store the parameters of circle's point in polar. */
struct circleInPolar {
    int a;
    int b;
    int r;
    circleInPolar() : a(0), b(0), r(0) {}
    circleInPolar(int _a, int _b, int _r) : a(_a), b(_b), r(_r) {}
    bool operator==(const circleInPolar cc) {
        return a == cc.a && b == cc.b && r == cc.r;
    }
};


/* Use for storing the peek imformation. */
struct peek {
    int a;
    int b;
    int r;
    int weight;
    peek() : a(0), b(0), r(0), weight(0) {}
    peek(int _a, int _b, int _r, int _weight) : a(_a), b(_b), r(_r), weight(_weight) {}
};


class HoughCircleTransform {
public:
    HoughCircleTransform(CImg<unsigned char> _image, int _rmin, int _rmax, float _threshold, float _disthreshold);
    void houghTransform();
    void findPeeks();
    void drawCircle();
    void getNumberOfCoins();
private:
    int rows;
    int cols;
    int rmin;
    int rmax;
    int maxVote;
    int hough_rows;
    int hough_cols;
    int hough_heis;
    int threshold;
    int disthreshold;
    int numberOfCoins;
    vector<float> cosPage;
    vector<float> sinPage;
    vector<peek> peekList;
    vector<vector<circleInPolar>> paperCircle;
    CImg<int> houghImage;
    CImg<unsigned char> image;
};


/* Constructor */
HoughCircleTransform::HoughCircleTransform(CImg<unsigned char> _image, int _rmin, int _rmax, float _threshold, float _disthreshold) {
    /* The edge image. */
    rows = _image._width;
    cols = _image._height;
    rmin = _rmin;
    rmax = _rmax;
    maxVote = 0;
    image = CImg<unsigned char>(rows, cols, 1, 3);
    image.fill(255);
    cimg_forXY(image, r, c) {
        if ((int)_image(r, c) == EDGE) {
            image(r, c, 0) = EDGE;
            image(r, c, 1) = EDGE;
            image(r, c, 2) = EDGE;
        }
    }
    image.display("I1", false);

    /* Hough image */
    hough_rows = rows;
    hough_cols = cols;
    hough_heis = (int)sqrt(rows*rows + cols*cols) / 2;
    houghImage.assign(hough_rows, hough_cols, hough_heis);
    houghImage.fill(0);
    //houghImage.display("houghImage");

    /* Create the angle set */
    for (int r = 0; r < 360; r += ANGLE_STEP) {
        float Cos = cos(2*M_PI*r/360);
        float Sin = sin(2*M_PI*r/360);
        cosPage.push_back(Cos);
        sinPage.push_back(Sin);
    }

    houghTransform();
    threshold = (maxVote - sqrt(maxVote)) * _threshold;
    disthreshold = _disthreshold;

    findPeeks();
    drawCircle();
    getNumberOfCoins();
}


/* Hough transform for circle */
void HoughCircleTransform::houghTransform() {
    /* Vote for each a b r */
    cimg_forXY(image, x, y) {
        if ((int)image(x, y, 0) == NOEDGE) continue;
        for (int r = rmin; r < rmax; r += RADIUS_STEP) {
            for (int d = 0; d < 360; d += ANGLE_STEP) {
                double a = x - r*cosPage[d];
                double b = y - r*sinPage[d];
                if (a >= 0 && a < rows && b >= 0 && b < cols && r >= 0 && r < hough_heis) {
                    houghImage((int)a, (int)b, r) = houghImage((int)a, (int)b, r) + 1;
                    int houghValue = houghImage((int)a, (int)b, r);
                    maxVote = (maxVote > houghValue) ? maxVote : houghValue;
                }
            }
        }
    }
    printf("MaxVote value: %d\n", maxVote);
    houghImage.display("HoughTransform");
}


/* Find the peek's point in some district */
void HoughCircleTransform::findPeeks() {
    cimg_forXYZ(houghImage, a, b, r) {
        int hough_value = houghImage(a, b, r);
        if (hough_value < threshold) continue;

        /* Match the threshold */
        bool isNewCircle = true;
        for (int m = 0; m < paperCircle.size(); m++) {
            int count = 0;
            for (int n = 0; n < paperCircle[m].size(); n++) {
                int ma = paperCircle[m][n].a;
                int mb = paperCircle[m][n].b;
                int mr = paperCircle[m][n].r;

                int da = ma - a;
                int db = mb - b;
                int dr = mr - r;

                int distance = (int)sqrt(da*da + db*db + dr*dr);
                if (distance <= disthreshold) {
                    count++;
                }
            }

            /* If count equals to paperCircle's size, it's the same point. */
            if (count == paperCircle[m].size()) {
                isNewCircle = false;
                paperCircle[m].push_back(circleInPolar(a, b, r));
                break;
            }            
        }

        /* Create a new circle. */
        if (isNewCircle) {
            vector<circleInPolar> newCircle;
            newCircle.push_back(circleInPolar(a, b, r));
            paperCircle.push_back(newCircle);
        }
    }

    /* Seek for the peak. */
    peekList = vector<peek>(paperCircle.size(), peek(0, 0, 0, 0));
    for (int m = 0; m < paperCircle.size(); m++) {
        for (int n = 0; n < paperCircle[m].size(); n++) {
            circleInPolar temp = paperCircle[m][n];
            int value = houghImage(temp.a, temp.b, temp.r);
            if (value >= peekList[m].weight) {
                //cout << m << " " << value << endl;
                peekList[m].a = temp.a;
                peekList[m].b = temp.b;
                peekList[m].r = temp.r;
                peekList[m].weight = value;
                //cout << m << " " << peekList[m].weight << endl;
            }
        }
    }

    /* Print the peek messages. */
    for (int i = 0; i < peekList.size(); i++) {
        printf("Circle%d: a=%d\tb=%d\tr=%d\tweight=%d\n", i, peekList[i].a, peekList[i].b, peekList[i].r, peekList[i].weight);
    }
}


/* Draw the circle. */
void HoughCircleTransform::drawCircle() {
    unsigned char red[] = {255, 0, 0};
    unsigned char blue[] = {0, 0, 255};

    for (int m = 0; m < peekList.size(); m++) {
        int centerX = peekList[m].a;
        int centerY = peekList[m].b;
        int radius = peekList[m].r;

        /* Fitting the circle. */
        image.draw_circle(centerX, centerY, radius, red, 5.0f, 1);
    }

    image.display("I2", false);

    /* Show the relative points of the circle. */
    /* Distance within 5 is seemed to the same circle. */
    int circleError = 5;
    for (int m = 0; m < peekList.size(); m++) {
        int centerX = peekList[m].a;
        int centerY = peekList[m].b;
        int radius  = peekList[m].r;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int dr = r - centerX;
                int dc = c - centerY;
                int dis = (int)sqrt(dr*dr + dc*dc);
                if (abs(dis-radius) <= circleError && image(r, c, 0) == 0 && image(r, c, 1) == 0 && image(r, c, 2) == 0) {
                    image(r, c, 0) = 0;
                    image(r, c, 1) = 0;
                    image(r, c, 2) = 255;
                }                
            }
        }
    }
    image.display("I3", false);
}


/* Get the number of coins. */
void HoughCircleTransform::getNumberOfCoins() {
    numberOfCoins = peekList.size();
    printf("The number of coins is: %d\n", numberOfCoins);
    image.save("Circle.bmp");
}


/* Main to test. */
int main(int argc, char *argv[]) {
    if(argc < 10){
        fprintf(stderr,"\n<USAGE> %s image sigma tlow thigh lenTodelete rmin rmax",argv[0]);
        fprintf(stderr,"\n                 threshold disthreshold [writedirim]\n");
        fprintf(stderr,"\n      image:        An image to process. Must be in ");
        fprintf(stderr,"PGM format.\n");
        fprintf(stderr,"      sigma:        Standard deviation of the gaussian");
        fprintf(stderr," blur kernel.\n");
        fprintf(stderr,"      tlow:         Fraction (0.0-1.0) of the high ");
        fprintf(stderr,"edge strength threshold.\n");
        fprintf(stderr,"      thigh:        Fraction (0.0-1.0) of the distribution");
        fprintf(stderr," of non-zero edge\n                    strengths for ");
        fprintf(stderr,"hysteresis. The fraction is used to compute\n");
        fprintf(stderr,"                    the high edge strength threshold.\n");
        fprintf(stderr,"      lenTodelete:  Delete edge which is less than len\n");
        fprintf(stderr,"      rmin:         the minimum radius of the hough space.\n");
        fprintf(stderr,"      rmax:         the maximum radius of the hough space.\n");
        fprintf(stderr,"      threshold:    the threshold that the votes should exceed.\n");
        fprintf(stderr,"      disthreshold: the threshold that judge the same circle.\n");
        fprintf(stderr,"      writedirim:   Optional argument to output ");
        fprintf(stderr,"a floating point");
        fprintf(stderr," direction image.\n\n");
        exit(1);
    }
    MyCanny canny(argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]));
    CImg<unsigned char> image = canny.getFinalImage();
    HoughCircleTransform houghCircleTransform(image, atof(argv[6]), atof(argv[7]), atof(argv[8]), atof(argv[9]));
    return 0;
}

#endif //_HOUGHCIRCLE_
