#include "stdafx.h"
#include "segmentation.h"


// Constructor of Segmentation.
Segmentation::Segmentation(string fileName) {
    printf("> Begin the segmentation of the image.\n");
    image = CImg<float>(fileName.c_str());
    CImg<float> greyImage = createGreyImage(image);
    // greyImage.display("Grey", false);
    vector<int> histogram = createHistogram(greyImage);

    // Display the graph of the histogram.
    CImg<int> histImg = greyImage.histogram(256, 0, 255);
    histImg.display_graph("Histogram", 3);

    vector<float> probability = createNormalizeHistogram(histogram);

    int threshold = createBestThreshold(probability, histogram);
    printf("> The best threshold of the image is: %d\n", threshold);

    CImg<float> result = segmentation(threshold);
    result.display("Segmentation", false);

    getDerrivative(result);
    edgeDetect(result);
}

Segmentation::~Segmentation() {
    printf("> Segmentation Finish! Thank you!\n");
}

// Create the grey image.
CImg<float> Segmentation::createGreyImage(CImg<float> _image) {
    printf("> Begin creating the grey image\n");
    CImg<float> t_image(image._width, image._height, 1, 1, 0);
    cimg_forXY(t_image, x, y) {
        t_image(x, y) = (image(x, y, 0) * 299 + image(x, y, 1) * 587 + image(x, y, 2) * 114 + 500) / 1000;
    }
    printf("> Creating grey image\n");
    return t_image;
}

// 1. Create the histogram of the image.
vector<int> Segmentation::createHistogram(CImg<float> greyImage) {
    printf("> Begin creating the histogram\n");
    vector<int> histogram(256, 0);
    cimg_forXY(greyImage, x, y) {
        histogram[(int)greyImage(x, y)]++;
    }
    printf("> Creating histogram completed\n");
    return histogram;
}

// 2. Normalize the histogram and get probability of each pixel.
vector<float> Segmentation::createNormalizeHistogram(vector<int> histogram) {
    printf("> Normalizing the histogram\n");
    vector<float> probability(256, 0.0);
    float sum = 0.0;
    for (int i = 0; i < 256; i++) {
        sum += histogram[i];
    }
    for (int i = 0; i < 256; i++) {
        probability[i] = histogram[i] / sum;
    }
    printf("> Normalizing the histogram completed\n");
    return probability;
}

// 3. Traverse the all the pixels and get the best threshold.
int Segmentation::createBestThreshold(vector<float> probability, vector<int> histogram) {
    printf("> Begin finding the best threshold\n");
    int threshold = 0;
    float Max = 0;
    for (int u = 0; u < 256; u++) {
        float w0 = 0;   // Weight of the foreward points'.
        float w1 = 0;   // Weight of the backward points'.
        int u0 = 0;     // Mean of the foreward points'.
        int u1 = 0;     // Mean of the backward points'.
        int sum_u0 = 0;
        int sum_u1 = 0;

        // Calculate the Foreground point's weight and means.
        for (int i = 0; i < u; i++) {
            sum_u0 += histogram[i];
            u0 += i * histogram[i];
            w0 += probability[i];
        }
        u0 = (sum_u0 == 0) ? 0 : u0 / sum_u0;
        //u0 /= sum_u0;

        // Calculate the Background point's weight and means.
        for (int i = u; i < 256; i++) {
            sum_u1 += histogram[i];
            u1 += i * histogram[i];
            w1 += probability[i];
        }
        u1 = (sum_u1 == 0) ? 0 : u1 / sum_u1;
        //u1 /= sum_u1;

        float g = w0 * w1 * (u0 - u1) * (u0 - u1);
        if (g > Max) {
            Max = g;
            threshold = u;
        }
        printf("> w0=%f w1=%f u0=%d u1=%d threshold=%d\n", w0, w1, u0, u1, threshold);
    }

    printf("> Finding best threshold completed\n");
    return threshold;
}

// 4. Get the final result according to the threshold.
CImg<float> Segmentation::segmentation(int threshold) {
    printf("> Start creating the final result\n");
    CImg<float> finalResult = createGreyImage(image);
    // finalResult.display("Test", false);
    cimg_forXY(finalResult, x, y) {
        finalResult(x, y, 0) = (finalResult(x, y) >= threshold) ? 0 : 255;
    }
    return finalResult;
}

// Get the derrivative of x-dirrection and y-dirrection.
void Segmentation::getDerrivative(CImg<float> result) {
    int rows = result._width;
    int cols = result._height;
    detX = vector<vector<int>>(rows, vector<int>(cols, 0));

    /* Compute the x-derivative filter = [-1, 0, +1] */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (c == 0) detX[r][c] = result(r, c+1) - result(r, c);
            else if (c == cols-1) detX[r][c] = result(r, c) - result(r, c-1);
            else detX[r][c] = result(r, c+1) - result(r, c-1);
        }
    }

    detY = vector<vector<int>>(rows, vector<int>(cols, 0));

    /* Compute the y-derivative filter = [-1, 0, +1]^T */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (r == 0) detY[r][c] = result(r+1, c) - result(r, c);
            else if (r == rows-1) detY[r][c] = result(r, c) - result(r-1, c);
            else detY[r][c] = result(r+1, c) - result(r-1, c);
        }
    }    
}

// Edge detect.
void Segmentation::edgeDetect(CImg<float> result) {
    int cols = result._width;
    int rows = result._height;
    finalResult = CImg<unsigned char>(cols, rows, 1, 1, 0);
    for (int c = 0; c < cols; c++) {
        for (int r = 0; r < rows; r++) {
            if (detX[c][r] != 0 || detY[c][r] != 0) 
                finalResult(c, r) = 0;
            else
                finalResult(c, r) = 255;
        }
    }
    finalResult.display("Edge Detect", false);
}

// Function for returning the result of the image.
CImg<unsigned char> Segmentation::getFinalResult() {
    return finalResult;
}