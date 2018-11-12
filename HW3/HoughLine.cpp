/*******************************************************************************
* PROGRAM : Computer Vision HW3 - Detect lines
* AUTHOR  : Liang Junhua
* DATE    : 2018/10/21
*******************************************************************************/

#ifndef _HOUGH_
#define _HOUGH_
#include "CImg.h"
#include "canny_source.h"
#include <set>
#define HOUGH_ROWS 500 //Divide the angle into some pieces.

#define EDGE 0
#define NOEDGE 255
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
    void drawCirclePoint();
    vector<pek> findPeeks(float thre, float dise);
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


/* Initialize the hough space. */
Hough::Hough(CImg<unsigned char> t_image, float thre, float dise) {
    /* The original image. */
    rows = t_image._width;
    cols = t_image._height;
    edgeImage = t_image;
    image = CImg<unsigned char>(rows, cols, 1, 3);
    image.fill(255);
    cimg_forXY(image, r, c) {
        if ((int)t_image(r, c) == EDGE) {
            image(r, c, 0) = EDGE;
            image(r, c, 1) = EDGE;
            image(r, c, 2) = EDGE;
        }
    }

    /* The hough image
       r ranges in [-D/2, D/2] 
       t ranges in [0, pi]    */
    max_vote = 0;
    radian = M_PI / HOUGH_ROWS;
    maxLength = 2 * (int)sqrt((rows/2)*(rows/2) + (cols/2)*(cols/2));
    hough_rows = HOUGH_ROWS;
    hough_cols = maxLength;
    Hough_space = CImg<int>(hough_rows, hough_cols, 1, 1);
    Hough_space.fill(0);

    hough_vote();
    votePeek = findPeeks(thre, dise);
    drawLine(thre, dise);
    findRelativeEdge();
    drawCirclePoint();

    image.save("Line.bmp");
}


/* Detect every line the edge point may go through and vote. */
void Hough::hough_vote() {
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            //If it's not an edge point.
            if ((int)image(r, c, 0) == NOEDGE) continue;

            //If it's an edge point.
            else {
                for (int t = 0; t < hough_rows; t++) {
                    double distance = (r - rows/2) * cos(t * radian) + (c - cols/2) * sin(t * radian);
                    distance += (maxLength / 2);
                    if (distance >= 0 && distance < hough_cols) {
                        Hough_space(t, (unsigned int)distance) = Hough_space(t, (unsigned int)distance) + 1;
                        if (max_vote < Hough_space(t, (unsigned int)distance)) max_vote = Hough_space(t, (unsigned int)distance);
                    }
                }
            }
        }
    }
    printf("Max_vote: %d\n", max_vote);
    Hough_space.display("Hough");
}


/* Find the point whose votes is the peek. */
vector<pek> Hough::findPeeks(float thre, float dise) {
    int threshold = (max_vote - sqrt(max_vote)) * thre;
    int diseshold = dise;
    cimg_forXY(Hough_space, t, r) {
        if (Hough_space(t, r) >= threshold) {
            bool isNewLine = true;

            /* Look whether it's neighbor to a peek. */
            for (int i = 0; i < paperLine.size(); i++) {
                int count = 0;
                vector<Line> temp = paperLine[i];
                for (int j = 0; j < paperLine[i].size(); j++) {
                    int mt = t;
                    int mr = r;
                    int tt = temp[j].t;
                    int rr = temp[j].r;

                    int dt = mt - tt;
                    int dr = mr - rr;

                    int dis = (int)sqrt(dt*dt + dr*dr);
                    if (dis <= diseshold) {
                        count++;
                    }
                    if (count == paperLine[i].size()) {
                        isNewLine = false;
                        paperLine[i].push_back(Line(t, r));
                        break;
                    }
                }

                if (!isNewLine) {
                    break;
                }
            }

            if (isNewLine) {
                vector<Line> temp;
                temp.push_back(Line(t, r));
                paperLine.push_back(temp);
            }
        }
    }

    /* Seek for the peak. */
    vector<pek> peekPoint(paperLine.size(), pek(0, 0, 0)); 
    for (int i = 0; i < paperLine.size(); i++) {
        for (int j = 0; j < paperLine[i].size(); j++) {
            int value = Hough_space(paperLine[i][j].t, paperLine[i][j].r);
            if (value >= peekPoint[i].peek) {
                //cout << value << endl;
                peekPoint[i].r = paperLine[i][j].r;
                peekPoint[i].t = paperLine[i][j].t;
                peekPoint[i].peek = value;
            }
        }
    }

    /* Sort the peek. */
    for (int i = 0; i < peekPoint.size(); i++) {
        for (int j = 0; j < peekPoint.size()-i-1; j++) {
            if (peekPoint[j].peek < peekPoint[j+1].peek) {
                int temp = peekPoint[j].peek;
                peekPoint[j].peek = peekPoint[j+1].peek;
                peekPoint[j+1].peek = temp;
            }
        }
    }

    return peekPoint;
}


/* Draw the line in the paper. */
void Hough::drawLine(float thre, float dise) {
    vector<vector<double>> lineFactor(4, vector<double>(3, 0));
    int threshold = (max_vote - sqrt(max_vote)) * thre;
    int diseshold = dise;

    /* Draw Line */
    for (int i = 0; i < votePeek.size(); i++) {
        pek tpek = votePeek[i];
        int rr = tpek.r;
        int tt = tpek.t;
        double Cos = cos(tt * radian);
        double Sin = sin(tt * radian);

        /* Judge whether it's a new line. */
        bool isNewLine = true;
        for (int m = 0; m < i; m++) {
            int mr = votePeek[m].r;
            int mt = votePeek[m].t;
            int dt = mt - tt;
            int dr = mr - rr;
            if (abs(dt) >= hough_rows - diseshold) {
                dt -= hough_rows;
                dr = abs(abs(mr-maxLength/2) - abs(rr-maxLength/2));
            }
            int dis = (int)sqrt(dt*dt + dr*dr);
            if (dis <= diseshold) {
                isNewLine = false;
                break;
            }
        }
        if (!isNewLine) continue;

        /* Print the line's equation in polar */
        double b = rr - maxLength/2 + rows/2*Cos + cols/2*Sin;
        printf("Line Equation: %lfx + %lfy = %lf\n", Cos, Sin, b);
        vector<double> temp {Cos, Sin, b};
        lineFactor.push_back(temp);

        /* Draw lines and find their intersection. */
        
        for (int x = 0; x < rows; x++) {
            for (int y = 0; y < cols; y++) {
                int dx = x - rows/2;
                int dy = y - cols/2;
                int dis1 = (rr - maxLength/2 - dx*Cos) / Sin + cols/2;
                int dis2 = (rr - maxLength/2 -dy*Sin) / Cos + rows/2;
                if (dis1 == y || dis2 == x) {
                    image(x, y, 0) = 0;
                    image(x, y, 1) = 0;
                    image(x, y, 2) = 255;

                    /* Judge whether the point is existing, if it's, it's an intersection. */
                    
                    auto it = linePoint.find(make_pair(x, y));
                    if (it != linePoint.end()) {
                        bool isNeighbor = false;
                        for (int p = 0; p < intersection.size(); p++) {
                            int px = intersection[p].first;
                            int py = intersection[p].second;
                            int qx = px - x;
                            int qy = py - y;
                            int dis = (int)sqrt(qx*qx + qy*qy);
                            if (dis <= 3) {
                                isNeighbor = true;
                                break;
                            }
                        }
                        if (!isNeighbor) intersection.push_back(make_pair(x, y));
                    } 
                    else if (x >= 0 && x < rows && y >= 0 && y < cols) {
                        linePoint.insert(make_pair(x, y));
                    }
                }
            }
        }
    }

    /*-----------------------Error---------------------
    for (int i = 0; i < lineFactor.size(); i++) {
        for (int j = i+1; j < lineFactor.size(); j++) {
            int a1 = lineFactor[i][0];
            int b1 = lineFactor[i][1];
            int c1 = lineFactor[i][2];
            int a2 = lineFactor[j][0];
            int b2 = lineFactor[j][1];
            int c2 = lineFactor[j][2];
            int mx = (b1*c2-b2*c1) / (a1*b2-b1*a2);
            int my = (a1*c2-a2*c1) / (a2*b1-a1*b2);
            intersection.push_back(make_pair(mx, my));
        }
    }
    ---------------------------------------------------*/

    /* Show all the intersection. */
    for (int i = 0; i < intersection.size(); i++) {
        printf("Intersection%d : x = %d, y = %d\n", i, intersection[i].first, intersection[i].second);
    }

    image.display("I2");
}


/* Find the point that is relative to the A4 paper */
void Hough::findRelativeEdge() {
    if (intersection.size() < 4) {
        fprintf(stderr, "There are less than four intersections.\n");
    }

    /* Red color. */
    unsigned char color[] = {255, 0, 0};

    /* Use CImg's algorithm */
    /* Sort intersection by x point. */
    for (int m = 0; m < intersection.size(); m++) {
        for (int n = 0; n < intersection.size()-m-1; n++) {
            if (intersection[n].first > intersection[n+1].first) {
                pair<int, int> temp = intersection[n];
                intersection[n] = intersection[n+1];
                intersection[n+1] = temp;
            }
        }
    }

    for (int i = 0; i < intersection.size(); i++) {
        cout << intersection[i].first << " " << intersection[i].second << endl;
    }

    int fromOne = intersection[0].first;
    int fromTwo = intersection[1].first;
    int fromThree = intersection[2].first;
    int fromFour = intersection[3].first;

    int toOne = intersection[0].second;
    int toTwo = intersection[1].second;
    int toThree = intersection[2].second;
    int toFour = intersection[3].second;

    image.draw_line(fromOne, toOne, fromTwo, toTwo, color);
    image.draw_line(fromThree, toThree, fromFour, toFour, color);

    /* Sort intersection by y point. */
    for (int m = 0; m < intersection.size(); m++) {
        for (int n = 0; n < intersection.size()-m-1; n++) {
            if (intersection[n].second > intersection[n+1].second) {
                pair<int, int> temp = intersection[n];
                intersection[n] = intersection[n+1];
                intersection[n+1] = temp;
            }
        }
    }
    fromOne = intersection[0].first;
    fromTwo = intersection[1].first;
    fromThree = intersection[2].first;
    fromFour = intersection[3].first;

    toOne = intersection[0].second;
    toTwo = intersection[1].second;
    toThree = intersection[2].second;
    toFour = intersection[3].second;

    image.draw_line(fromOne, toOne, fromTwo, toTwo, color);
    image.draw_line(fromThree, toThree, fromFour, toFour, color);

    image.display("I3");
}


/* Draw circle with radius 5px in intersection points. */
void Hough::drawCirclePoint() {
    /* Green color */
    unsigned char color[] = {0, 255, 0};

    for (int i = 0; i < intersection.size(); i++) {
        image.draw_circle(intersection[i].first, intersection[i].second, 5, color);
    }

    image.display("I4");
}


/* Main to test. */
int main(int argc, char *argv[]) {
    if(argc < 8){
        fprintf(stderr,"\n<USAGE> %s image sigma tlow thigh lenToDelete threshold disthreshold [writedirim]\n",argv[0]);
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
        fprintf(stderr,"      lenToDelete:  Delete edge which is less than len\n");
        fprintf(stderr,"      threshold:    the threshold that the votes should exceed.\n");
        fprintf(stderr,"      disthreshold: the threshold that judge the same line.\n");
        fprintf(stderr,"      writedirim:   Optional argument to output ");
        fprintf(stderr,"a floating point");
        fprintf(stderr," direction image.\n\n");
        exit(1);
    }
    MyCanny canny(argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]));
    CImg<unsigned char> image = canny.getFinalImage();
    image.display("I_edge");
    Hough hough(image, atof(argv[6]), atof(argv[7]));
    return 0;
}

#endif //_HOUGH_


