#include "stdafx.h"

#include <iostream>
#include "Hough.h"
#include "segmentation.h"
#include "wraping.h"


// Line1 : 0.4 20
// Line2 : 0.6 20
// Line3 : 0.6 20
// Line4 : 0.4 20
// Line5 : 0.2 20
// Line6 : 0.4 20
int main(int argc, char* argv[])
{
	string filename = "./Line6.bmp";
	CImg<float> image = CImg<float>(filename.c_str());
	Segmentation s(filename);
	CImg<unsigned char> edgeImage = s.getFinalResult();
	Hough hough(edgeImage, 0.4, 20);
	
	// Get the four intersection of the A4 paper;
	vector<pair<int, int>> intersection = hough.getIntersection();

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3-i; j++) {
			if (intersection[j].first > intersection[j+1].first) {
				pair<int, int> temp = intersection[j];
				intersection[j] = intersection[j+1];
				intersection[j+1] = temp;
			}
		}
	}

	if (intersection[0].second > intersection[1].second) {
		pair<int, int> temp = intersection[0];
		intersection[0] = intersection[1];
		intersection[1] = temp;
	}

	if (intersection[2].second > intersection[3].second) {
		pair<int, int> temp = intersection[2];
		intersection[2] = intersection[3];
		intersection[3] = temp;
	}

	pair<int, int> temp = intersection[1];
	intersection[1] = intersection[2];
	intersection[2] = temp;

	// Show the four intersections.
	for (int i = 0; i < 4; i++) {
		printf("> Point %d: (%d, %d)\n", i+1, intersection[i].first, intersection[i].second);
	}

	CImgList<float> TransferMatrix = createTransferMatrix(intersection, image);
	CImg<float> result = Warphing(image, TransferMatrix);
	result.display("Warphing A4 Paper", false);
	return 0;
}

