/*******************************************************************************
* PROGRAM : Computer Vision HW2
* AUTHOR  : Liang Junhua
* DATE    : 2018/10/14
*******************************************************************************/
#include "canny_source.h"


/* Construct function for the canny. */
MyCanny::MyCanny(char* filename, float sigma, float tlow, float thigh, int len) {
    /* Look what type of image */
    int length = strlen(filename);
    char name[5] = {0};
    name[0] = filename[length-4];
    name[1] = filename[length-3];
    name[2] = filename[length-2];
    name[3] = filename[length-1];

    /* Read the bmp image */
    if (!read_pgm_image(filename)) {
        fprintf(stderr, "Invalid filename %s input.\n", filename);
    }
    else {
        printf("Reading the image %s.\n", filename);
    }

    printf("Starting Canny edge detection.\n");

    /* Create the grey image. */
    if (strncmp(name, ".bmp", 4) == 0) {
        printf("Creating the Gray Scale Image.\n");
        pgm_image = this->createGrayImage();
    }


    /* Create the gauss kernel. */
    printf("Creating the gauss kernel.\n");
    gaussKernel = this->createGaussKernel(sigma);

    /* Gaussian smooth */
    printf("Smoothing the image using a gaussian kernel.\n");
    this->gaussian_smooth(sigma);

    /* Compute the derivative of the x-direction and y-direction. */
    printf("Computing the X and Y first derivatives.\n");
    delta_x = this->derrivative_x();
    delta_y = this->derrivative_y();

    /* Compute the direction of the image. */
    radians = this->radian_direction(0, 0);

    /* Compute the magnitude of the image. */
    printf("Computing the magnitude of the gradient.\n");
    magnitude = this->magnitude_x_y();

    /* Non-maximum-suppress. */
    printf("Doing the non-maximal suppression.\n");
    this->non_max_supp();

    /* Apply hysteresis. */
    this->apply_hysteresis(tlow, thigh);

    /* Edge linking. */
    this->edge_linking(len);

    this->write_pgm_image(filename, sigma, tlow, thigh, name);
}


/* Gauss Function with standard deviation sigma. */
float MyCanny::gaussFunction(float sigma, float x) {
    return pow(2.71828, -0.5*x*x/(sigma*sigma)) / (sigma * sqrt(6.2831853));
}


/* Create a Gauss Kernel. */
vector<float> MyCanny::createGaussKernel(float sigma) {
    float x, fx, sum = 0.0;
    int windowsize = 1 + 2 * ceil(2.5 * sigma);
    int center = windowsize / 2;
    vector<float> kernel(windowsize, 0);

    for (int i = 0; i < windowsize; i++) {
        x = (float)(i - center);
        fx = gaussFunction(sigma, x);
        kernel[i] = fx;
        sum += fx;
    }

    /* Normalize the matrix. */
    for (int i = 0; i < windowsize; i++) {
        kernel[i] /= sum;
    }

    return kernel;
}


/* Function of Gauss blur. */
void MyCanny::gaussian_smooth(float sigma) {
    CImg<short int> gaussian_result(pgm_image._width, pgm_image._height, 1);
    int rows = pgm_image._width;
    int cols = pgm_image._height;
    float sum = 0.0;    /* Sum of the kernel weights variable. */
    float dot = 0.0;    /* Dot product summing variable. */
    int windowsize = 1 + 2 * ceil(2.5 * sigma);
    int center = windowsize / 2;
    vector<float> tempim(rows*cols, 0);

    /* Blur in the x-direction */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            dot = 0.0;
            sum = 0.0;
            for (int cc = -center; cc <= center; cc++) {
                if (((c+cc) >= 0) && ((c+cc) < cols)) {
                    dot += (float)pgm_image(r, c+cc) * gaussKernel[center+cc];
                    sum += gaussKernel[center+cc];
                }
            }
            tempim[r*cols+c] = dot/sum;
        }
    }

    /* Blur in the y-direction */
    for (int c = 0; c < cols; c++) {
        for (int r = 0; r < rows; r++) {
            dot = 0.0;
            sum = 0.0;
            for (int rr = -center; rr <= center; rr++) {
                if (((r+rr) >= 0) && ((r+rr) < rows)) {
                    dot += tempim[(r+rr)*cols+c] * gaussKernel[center+rr];
                    sum += gaussKernel[center+rr];
                }
            }
            gaussian_result(r, c) = (short int)(dot*BOOSTBLURFACTOR/sum + 0.5);
        }
    }

    /* Display the origin image and after gauss blur */
    gaussian_result.display("Gaussian Blur");
    gaussian_smooth_image = gaussian_result;
}


/* Create a grey image. */
CImg<unsigned char> MyCanny::createGrayImage() {
    CImg<unsigned char> greyImage(pgm_image._width, pgm_image._height, 1);
    cimg_forXY(pgm_image, x, y) {
        /* Turn RGB to grey. */
        
        double greyValue = (0.299 * pgm_image(x, y ,0))
        + (0.587 * pgm_image(x, y, 1))
        + (0.114 * pgm_image(x, y, 2));

        greyImage(x, y) = greyValue;
    }

    greyImage.display("Gray image");
    return greyImage;
}


/* Calculate the derrivative of x-direction */
vector<vector<short int>> MyCanny::derrivative_x() {
    int rows = gaussian_smooth_image._width;
    int cols = gaussian_smooth_image._height;
    vector<vector<short int>> detx(rows, vector<short int>(cols, 0));

    /* Compute the x-derivative filter = [-1, 0, +1] */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (c == 0) detx[r][c] = gaussian_smooth_image(r, c+1) - gaussian_smooth_image(r, c);
            else if (c == cols-1) detx[r][c] = gaussian_smooth_image(r, c) - gaussian_smooth_image(r, c-1);
            else detx[r][c] = gaussian_smooth_image(r, c+1) - gaussian_smooth_image(r, c-1);
        }
    }

    return detx;
}


/* Calculate the derrivative of y-direction */
vector<vector<short int>> MyCanny::derrivative_y() {
    int rows = gaussian_smooth_image._width;
    int cols = gaussian_smooth_image._height;
    vector<vector<short int>> dety(rows, vector<short int>(cols, 0));

    /* Compute the y-derivative filter = [-1, 0, +1]^T */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (r == 0) dety[r][c] = gaussian_smooth_image(r+1, c) - gaussian_smooth_image(r, c);
            else if (r == rows-1) dety[r][c] = gaussian_smooth_image(r, c) - gaussian_smooth_image(r-1, c);
            else dety[r][c] = gaussian_smooth_image(r+1, c) - gaussian_smooth_image(r-1, c);
        }
    }

    return dety;
}


/* Compute the magnitude of the gradient. */
vector<vector<short int>> MyCanny::magnitude_x_y() {
    int rows = gaussian_smooth_image._width;
    int cols = gaussian_smooth_image._height;
    vector<vector<short int>> mag(rows, vector<short int>(cols, 0));

    /* The square root of the sum of the squared derivative valuse. */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int sq1 = delta_x[r][c] * delta_x[r][c];
            int sq2 = delta_y[r][c] * delta_y[r][c];
            mag[r][c] = (short int)(0.5 + sqrt((float)sq1 + (float)sq2));
        }
    }

    return mag;
}


/* Compute a direction of the gradient image 
 *and choose the ways for calculating dx and dy. */
vector<vector<float>> MyCanny::radian_direction(int xdirtag, int ydirtag) {
    int rows = gaussian_smooth_image._width;
    int cols = gaussian_smooth_image._height;
    vector<vector<float>> radian(rows, vector<float>(cols, -1));

    /* Compute the direction. angle = atan(dy/dx) */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            double dx = (double)delta_x[r][c];
            double dy = (double)delta_y[r][c];

            /* FOR X: xdirtag = -1 for [-1 0 1]
                      xdirtag =  1 for [1 0 -1] 
               FOR Y: ydirtag =  1 for [-1 0 1]
                      ydirtag = -1 for [1 0 -1] */
            if (xdirtag == 1)  dx = -dx;
            if (ydirtag == -1) dy = -dy;

            radian[r][c] = (float)angle_radians(dx, dy);
        }
    }

    return radian;
}


/* This Function computes the angle of a vector with components x and y. */
double MyCanny::angle_radians(double x, double y) {
    double xu, yu, ang;
    xu = fabs(x);
    yu = fabs(y);

    if ((xu == 0) && (yu == 0)) return 0;

    ang = atan(yu/xu);

    /* Put the points in the correct quadrant. */
    if (x >= 0) {
        if (y >= 0) return ang;
        else return (2*M_PI - ang);
    }
    else {
        if (y >= 0) return (M_PI - ang);
        else return (M_PI + ang);
    }
}


/* Traces edges along all paths whose magnitude values 
   remain above some specifyable lower threshhold */
void MyCanny::follow_edges(CImg<unsigned char>& edgemap, int r, int c, int lowval) {
    /* Eight neighborhood tracking algorithm */
    /* r,c the current position. */
    int x[8] = {1,1,0,-1,-1,-1,0,1},
        y[8] = {0,1,1,1,0,-1,-1,-1};

    for (int i = 0; i < 8; i++) {
        if ((edgemap(r+x[i], c+y[i]) == POSSIBLE_EDGE) && magnitude[r+x[i]][c+y[i]] > lowval) {
            edgemap(r+x[i], c+y[i]) =  EDGE;
            follow_edges(edgemap, r+x[i], c+y[i], lowval);
        }
    }
}


/* non-maximal suppression to the magnitude of the gradient image */
void MyCanny::non_max_supp() {
    int rows = pgm_image._width;
    int cols = pgm_image._height;
    int grad1,grad2,grad3,grad4;
    int gradient_value;
    int detX,detY;
    double weight;
    double cmpValue1,cmpValue2;

    /* The result matrix. */
    CImg<unsigned char> result(rows, cols, 1, 1, 1);

    /* Zero the edges of the result image */
    for (int c = 0; c < cols; c++) {
        result(0, c) = (unsigned char)0;
        result(rows-1, c) = (unsigned char)0;
    }
    for (int r = 0; r < rows; r++) {
        result(r, 0) = (unsigned char)0;
        result(r, cols-1) = (unsigned char)0;
    }

    /* Suppress non-maximum points */
    for (int r = 1; r < rows-1; r++) {
        for (int c = 1; c < cols-1; c++) {
            /* If gradient is zero , it cannot be an edge*/
            if (magnitude[r][c] == 0) {
                result(r, c) = (unsigned char)NOEDGE;
            }
            else {
                /* The gradient of current point. */
                gradient_value = magnitude[r][c];

                /* The derrivation of x-direction and y-direction */
                detX = delta_x[r][c];
                detY = delta_y[r][c];
                //cout << "detX: " << detX << endl;
                //cout << "detY: " << detX << endl;

                /* fabs(detY) > fabs(detX) */
                if (abs(detY) > abs(detX)) {
                    /* Calculate the factor of interplation. */
                    weight = fabs(detX) / (fabs(gradient_value));
                    grad2 = magnitude[r-1][c];
                    grad4 = magnitude[r+1][c];

                    /*******************
                     * Case0           *
                     * g1  g2          *
                     *      C          *
                     *     g4  g3      *
                     *******************/
                    if (detX * detY > 0) {
                        grad1 = magnitude[r-1][c-1];
                        grad3 = magnitude[r+1][c+1];
                    }

                    /*******************
                     * Case1           *
                     *     g2  g1      *
                     *      C          *
                     * g3  g4          *
                     *******************/
                    else if (detX * detY <= 0) {
                        grad1 = magnitude[r-1][c+1];
                        grad3 = magnitude[r+1][c-1];
                    }
                }

                /* fabs(detY) <= fabs(detX) */
                else if (abs(detY) <= abs(detX)) {
                    /* Calculate the factor of interplation. */
                    weight = fabs(detY) / (fabs(gradient_value));
                    grad2 = magnitude[r][r+1];
                    grad4 = magnitude[r][c-1];

                    /*******************
                     * Case3           *
                     * g3              *
                     * g4  C  g2       *
                     *        g1       *
                     *******************/
                    if (detX * detY > 0) {
                        grad1 = magnitude[r+1][c+1];
                        grad3 = magnitude[r-1][c-1];
                    }

                    /*******************
                     * Case4           *
                     * g1              *
                     * g4  C  g2       *
                     *        g3       *
                     *******************/
                    else if (detX * detY <= 0) {
                        grad1 = magnitude[r-1][c-1];
                        grad3 = magnitude[r+1][c+1];
                    }
                }

                /* Judge whether the point is an EDGE */
                cmpValue1 = weight*grad1 + (1-weight)*grad2;
                cmpValue2 = weight*grad3 + (1-weight)*grad4;

                if (gradient_value >= cmpValue1 && gradient_value >= cmpValue2) {
                    /* It's POSSIBLE EDGE */
                    result(r, c) = (unsigned char)POSSIBLE_EDGE;
                }
                else {
                    /* Suppress */
                    result(r, c) = (unsigned char)NOEDGE;
                }
            }
        }
    }

    result.display("non_max_image");
    non_max_image = result;
}


/*******************************************************************************
* Hysteresis, find edges that are above some high threshold or
* are connected to a high pixel by a path of pixels greater than
* a low threshold
*******************************************************************************/
void MyCanny::apply_hysteresis(float tlow, float thigh) {
    int rows = pgm_image._width;
    int cols = pgm_image._height;
    int histogram[32768] = {0};
    int numedges = 0;
    int maximum_mag = 0;
    int highcount = 0;
    int highthreshold, lowthreshold;
    CImg<unsigned char> edge = non_max_image;

    /****************************************************************************
    * Compute the histogram of the magnitude image. Then use the histogram to
    * compute hysteresis thresholds.
    ****************************************************************************/
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if ((int)edge(r, c) == POSSIBLE_EDGE) {
                histogram[magnitude[r][c]]++;
            }
        }
    }

    /* Compute the number of pixels that passed the nonmaximal suppression. */
    for (int r = 1; r < 32768; r++) {
        /* Find the largest magnitude of the image. */
        if (histogram[r] != 0) maximum_mag = r;
        /* The number of pixels that pass the nonmaximal suppression. */
        numedges += histogram[r];
    }
    highcount = (int)(numedges * thigh + 0.5);

    /* Calculate the highthreshold and lowthreshold. */
    int r = 1;
    numedges = histogram[1];
    while ((r < (maximum_mag-1)) && (numedges < highcount)) {
        r++;
        numedges += histogram[r];
    }
    highthreshold = r;
    lowthreshold = (int)(highthreshold * tlow + 0.5);

    /* Looks for pixels above the highthreshold to locate edges */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (((int)edge(r, c) == POSSIBLE_EDGE) && (magnitude[r][c] >= highthreshold)) {
                edge(r, c) = EDGE;
                follow_edges(edge, r, c, lowthreshold);
            }
        }
    }

    /* Set all the remaining possible edges to non-edges. */
    for (int r = 0; r < rows; r++) {
        for (int c = 0 ; c < cols; c++) {
            if ((int)edge(r, c) != EDGE) edge(r, c) = NOEDGE;
        }
    }

    edge.display("hysteresis");
    edge_image = edge;
}


/* Edge linking and delete the edge which length is larger than 20. */
void MyCanny::edge_linking(int len) {
    int rows = edge_image._width;
    int cols = edge_image._height;
    CImg<unsigned char> final_result = edge_image;
    
    for (int r = 2; r < rows-2; r++) {
        for (int c = 2; c < cols-2; c++) {
            int count = 0;
            if ((int)final_result(r,c) == EDGE) {
                for (int rr = -1; rr < 2 ; rr++) {
                    for (int cc = -1; cc < 2; cc++) {
                        if (rr != 0 && cc != 0 
                        && (int)final_result(r+rr,c+cc) == EDGE) {
                            count++;
                        }
                    }
                }

                /* Search it's 16 neightbor */
                if (count == 1) {
                    for (int rr = -2; rr < 3; rr++) {
                        for (int cc = -2; cc < 3; cc++) {
                            if (!(rr < 2 && rr > -2 && cc < 2 && cc > -2) 
                            && (int)final_result(r+rr, c+cc) == EDGE) {
                                final_result(r+rr/2, c+cc/2) = EDGE;
                            }
                        }
                    }
                }
            }
        }
    }

    /* Show the image after edge linking. */
    final_result.display("Link result");

    /* Delete the edge which length is larger than 20. */
    int dx[8] = {1,1,0,-1,-1,-1,0,1},
        dy[8] = {0,1,1,1,0,-1,-1,-1};
    int length = 0;
    int pos = 0;
    queue<pair<int, int>> Queue;
    vector<vector<bool>> isVisited(rows, vector<bool>(cols, false));
    vector<stack<pair<int, int>>> Stack(1000, stack<pair<int, int>>());
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if ((int)final_result(r, c) == NOEDGE) {
                isVisited[r][c] = true;
            }
        }
    }

    /* BFS algorithm. */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            length = 0;
            if (!isVisited[r][c]) {
                Queue.push(make_pair(r, c));
                Stack[pos].push(make_pair(r, c));
                while (!Queue.empty()) {
                    int Row = Queue.front().first;
                    int Col = Queue.front().second;
                    if (!isVisited[Row][Col]) {
                        isVisited[Row][Col] = true;
                        length++;
                        Stack[pos].push(make_pair(Row,Col));
                        for (int i = 0; i < 8; i++) {
                            if (Row+dx[i] >= 0 && Row+dx[i] < rows &&
                                Col+dy[i] >= 0 && Col+dy[i] < cols &&
                                !isVisited[Row+dx[i]][Col+dy[i]]) {
                                Queue.push(make_pair(Row+dx[i],Col+dy[i]));
                                //Stack.push(make_pair(Row+dx[i],Col+dy[i]));
                            }
                        }
                    }
                    Queue.pop();
                }

                /* Delete the edge whose length is less than length. */
                if (length < len) {
                    while (!Stack[pos].empty()) {
                        int Row = Stack[pos].top().first;
                        int Col = Stack[pos].top().second;
                        final_result(Row, Col) = NOEDGE;
                        isVisited[Row][Col] = true;
                        Stack[pos].pop();
                    }
                }
                pos++;
            }
        }
    }

    /* Show the image after deleting. */
    final_result.display("Final result");

    edge_linking_image = final_result;
}


/* Read a bmp image and create the gauss kernel and the grey image. */
bool MyCanny::read_pgm_image(char* filename) {
    CImg<unsigned char> image(filename);
    pgm_image = image;
    pgm_image.display("pgm_image");
    return true;
}


/* Write a image in pgm format. */
bool MyCanny::write_pgm_image(char* filename, float sigma, float tlow, float thigh, char name[5]) {
    string Stigma,Tlow,Thigh;
    stringstream ss;
    ss << sigma;
    ss >> Stigma;
    ss.clear();

    ss << tlow;
    ss >> Tlow;
    ss.clear();

    ss << thigh;
    ss >> Thigh;
    ss.clear();

    string outfilename = filename;
    outfilename = outfilename.substr(0, outfilename.size()-4);
    outfilename = outfilename + "_" + Stigma + "_" + Tlow + "_" + Thigh;
    outfilename += name;

    /* save the image in format name + sigma + tlow + tight */
    printf("Writing the edge iname in the file %s.\n", outfilename.c_str());
    edge_linking_image.save(outfilename.c_str());
    return true;
}


/* Main to test. */
int main(int argc, char *argv[]) {
    if(argc < 6){
        fprintf(stderr,"\n<USAGE> %s image sigma tlow thigh [writedirim]\n",argv[0]);
        fprintf(stderr,"\n      image:      An image to process. Must be in ");
        fprintf(stderr,"PGM format.\n");
        fprintf(stderr,"      sigma:      Standard deviation of the gaussian");
        fprintf(stderr," blur kernel.\n");
        fprintf(stderr,"      tlow:       Fraction (0.0-1.0) of the high ");
        fprintf(stderr,"edge strength threshold.\n");
        fprintf(stderr,"      thigh:      Fraction (0.0-1.0) of the distribution");
        fprintf(stderr," of non-zero edge\n                  strengths for ");
        fprintf(stderr,"hysteresis. The fraction is used to compute\n");
        fprintf(stderr,"                  the high edge strength threshold.\n");
        fprintf(stderr,"      len:        Delete edge which is less than len\n");
        fprintf(stderr,"      writedirim: Optional argument to output ");
        fprintf(stderr,"a floating point");
        fprintf(stderr," direction image.\n\n");
        exit(1);
    }
    MyCanny canny(argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]));
    return 0;
}