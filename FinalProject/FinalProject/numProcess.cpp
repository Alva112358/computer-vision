#include "stdafx.h"
#include "numProcess.h"
#include "canny_source.h"

Process::Process(CImg<float> _source, int threshold, int S_value, int connect_value) {
	// Basic elements.
	source = CImg<float>(_source);
	rows = source._height;
	cols = source._width;
	this->threshold = threshold;
	this->connect_value = connect_value;

	// MyCanny mycanny(source, 4.0, 0.6, 0.8);
	// greyImage = mycanny.getCanny();
	// binaryImage = mycanny.getCanny();

	greyImage = CImg<float>(cols, rows, 1, 1, 0);
	binaryImage = CImg<float>(cols, rows, 1, 1, 0);

	// Image process.
	createGreyImage();
	// createBinaryImage();
	createBinaryImageByBernson(5, S_value, 128);
	// deleteExtraPoint(300, false);
	deleteExtraPoint(2, true);

	// Display part.
	greyImage.display("GreyImage", false);
	binaryImage.display("Binary", false);
	// binaryImage.save("./MiddleResult.bmp");
}

void Process::createGreyImage() {
	cimg_forXY(greyImage, x, y) {
		greyImage(x, y) = (source(x, y, 0) * 299 + source(x, y, 1) * 587 + source(x, y, 2) * 114 + 500) / 1000;
	}
}

void Process::createBinaryImage() {
	cimg_forXY(binaryImage, r, c) {
		if (r <= Binary_bound || c <= Binary_bound
			|| r >= cols - Binary_bound || c >= rows - Binary_bound) {
			binaryImage(r, c) = 255;
		}
		else binaryImage(r, c) = (greyImage(r, c) < this->threshold) ? 0 : 255;
	}
}

void Process::createBinaryImageByBernson(int size, int S, int Th) {
	binaryImage = CImg<float>(cols, rows, 1, 1, 0);
	CImg<float> src = greyImage;
	cimg_forXY(src, x, y) {
		float pixelMin = 255;
		float pixelMax = 0;
		float pixel = src(x, y);
		for (int i = x-size; i <= x+size; i++) {
			for (int j = y-size; j <= y+size; j++) {
				if ((i >= 0 && i < cols) && (j >= 0 && j < rows)) {
					pixelMin = pixelMin < src(i, j) ? pixelMin : src(i, j);
					pixelMax = pixelMax > src(i, j) ? pixelMax : src(i, j);					
				}
			}
		}
		
		float averagePixel = (pixelMax + pixelMin) / 2;
		if ((pixelMax - pixelMin) > S) {
			if (pixel > averagePixel) 
				binaryImage(x, y) = 255;
			else
				binaryImage(x, y) = 0;
		} else {
			if (averagePixel > Th)
				binaryImage(x, y) = 255;
			else
				binaryImage(x, y) = 0;
		}
	}

	cimg_forXY(binaryImage, r, c) {
		if (r <= Binary_bound || c <= Binary_bound
			|| r >= cols - Binary_bound || c >= rows - Binary_bound) {
			binaryImage(r, c) = 255;
		}
	}

	binaryImage.display("Bernson", false);
}

void Process::deleteExtraPoint(int len, bool less) {
	int cols = this->rows;
	int rows = this->cols;
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
            if ((int)binaryImage(r, c) == NOEDGE) {
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
                if (less && length <= len) {
					// cout << len << endl;
                    while (!Stack[pos].empty()) {
                        int Row = Stack[pos].top().first;
                        int Col = Stack[pos].top().second;
                        binaryImage(Row, Col) = 255;
                        isVisited[Row][Col] = true;
                        Stack[pos].pop();
                    }
                }

                else if (!less && length > len) {
					// cout << len << endl;
                    while (!Stack[pos].empty()) {
                        int Row = Stack[pos].top().first;
                        int Col = Stack[pos].top().second;
                        binaryImage(Row, Col) = 255;
                        isVisited[Row][Col] = true;
                        Stack[pos].pop();
                    }                	
                }
                pos++;
            }
        }
    }
}

// Find the segmentation point, the middle point is the dividing line.
void Process::findDividingLine() {
	int lineColor[3]{255, 0, 0};
	HistogramImage = CImg<float>(cols, rows, 1, 3, 255);
	DividingImg = CImg<float>(cols, rows, 1, 3, 255);
	vector<int> inflection_points;

	cimg_forY(HistogramImage, y) {
		int blackPixel = 0;

		// 统计x方向上的黑色像素
		cimg_forX(binaryImage, x) {
			DividingImg(x, y, 0) = binaryImage(x, y);
			DividingImg(x, y, 1) = binaryImage(x, y);
			DividingImg(x, y, 2) = binaryImage(x, y);
			if (binaryImage(x, y) == 0) {
				blackPixel++;
			}
		}

		// 生成直方图
		cimg_forX(HistogramImage, x) {
			if (x < blackPixel) {
				HistogramImage(x, y, 0) = 0;
				HistogramImage(x, y, 1) = 0;
				HistogramImage(x, y, 2) = 0;
			}
		}

		// 判断由黑到白或者由白到黑的拐点
		if (y > 0) {
			// 下白上黑
			if (blackPixel <= 0 && HistogramImage(0, y-1, 0) == 0) {
				inflection_points.push_back(y);
			}
			// 下黑上白
			else if (blackPixel > 0 && HistogramImage(0, y-1, 0) != 0) {
				inflection_points.push_back(y-1);
			}
		}
	}

	dividePoints.push_back(make_pair(0, -1));
	// 取中间值作为切割线
	if (inflection_points.size() > 2) {
		for (int i = 1; i < inflection_points.size()-1; i += 2) {
			// if (inflection_points[i+1]-inflection_points[i] < Row_threshold) continue;
			int divide = (inflection_points[i]+inflection_points[i+1]) / 2;
			dividePoints.push_back(make_pair(0, divide));
		}
	}
	dividePoints.push_back(make_pair(0, binaryImage._height-1));

	// 画直线检验
	for (int i = 0; i < dividePoints.size(); i++) {
		HistogramImage.draw_line(0, dividePoints[i].second, HistogramImage._width-1, dividePoints[i].second, lineColor);
		DividingImg.draw_line(0, dividePoints[i].second, DividingImg._width-1, dividePoints[i].second, lineColor);
	}

	HistogramImage.display("HistogramImg", false);
	// HistogramImage.save("./MiddleResult/HistogramImg.bmp");
	// DividingImg.display("Row segmentation", false);
	// DividingImg.save("./MiddleResult/RowResult.bmp");

}

void Process::divideIntoBarItemImg() {
	int lineColor[3]{ 255, 0, 0 };
	int Row_threshold = 20;
	vector<pair<int, int>> rows_line_img;
	// 对于Y方向的每个区域，进行X方向的切割
	for (int i = 1; i < dividePoints.size(); i++) {
		// 行分割的高度
		int barHeight = dividePoints[i].second - dividePoints[i-1].second;
		if (barHeight <= Row_threshold) {
			continue;
		} 
		if (i+1 < dividePoints.size() && (dividePoints[i+1].second - dividePoints[i].second <= Row_threshold)) {
			barHeight += (dividePoints[i+1].second - dividePoints[i].second);
		}
		int blackPixel = 0;
		// 定义一个行分割区域，准备进行列分割
		CImg<float> barItemImg = CImg<float>(binaryImage._width, barHeight, 1, 1, 0);
		cimg_forXY(barItemImg, x, y) {
			barItemImg(x, y) = binaryImage(x, dividePoints[i-1].second+1+y);
			if (barItemImg(x, y) == 0) {
				blackPixel++;
			}
		}
		double double_percent = (double)blackPixel / (double)(binaryImage._width * barHeight);
		// 只有当黑色像素的比例超过一定程度时，才能看作是有效的行分割结果.
		
		if (double_percent > 0.001) {
			vector<int> divideXset = getSubImageX(barItemImg);
			
			vector<CImg<float>> rowImgSet = getRowImg(barItemImg, divideXset);

			for (int j = 0; j < rowImgSet.size(); j++) {
				subImageSet.push_back(rowImgSet[j]);
				rows_line_img.push_back(make_pair(divideXset[j], dividePoints[i-1].second));
			}

			rowSize.push_back(rowImgSet.size());

			if (i > 1) {
				HistogramImage.draw_line(0, dividePoints[i-1].second, HistogramImage._width-1, dividePoints[i-1].second, lineColor);
				DividingImg.draw_line(0, dividePoints[i-1].second, HistogramImage._width-1, dividePoints[i-1].second, lineColor);
			}

			for (int j = 1; j < divideXset.size()-1; j++) {
				DividingImg.draw_line(divideXset[j], dividePoints[i-1].second, divideXset[j], dividePoints[i-1].second+barHeight, lineColor);
			}
		}
	}

	DividingImg.display("DividingImg", false);
	// DividingImg.save("./MiddleResult/DividingImgResult.bmp");

	dividePoints.clear();
	for (int i = 0; i < rows_line_img.size(); i++) {
		dividePoints.push_back(rows_line_img[i]);
	}

	ofstream outputFile("Row.txt");
	for (int i = 0; i < rowSize.size(); i++) {
		outputFile << rowSize[i] << endl;
	}
}

bool cou = true;
vector<int> Process::getSubImageX(CImg<float>& barItem) {
	// 绘制X方向的灰度直方图
	CImg<float> XHistogramImage = CImg<float>(barItem._width, barItem._height, 1, 3, 255);
	cimg_forX(barItem, x) {
		int blackPixel = 0;
		cimg_forY(barItem, y) {
			if (barItem(x, y) == 0) {
				blackPixel++;
			}
			// 生成X方向的直方图
			if (blackPixel >= 4) {
				cimg_forY(barItem, y) {
					if (y < blackPixel) {
						XHistogramImage(x, y, 0) = 0;
						XHistogramImage(x, y, 1) = 0;
						XHistogramImage(x, y, 2) = 0;						
					}
				}
			}
		}
	}

	int lastBlackPixel = -1;
	int pixelThreshold = connect_value;
	bool isFirstBlackPixel = false;
	// 在阈值以内的白色像素点都会被认为是黑色像素点
	for (int x = 0; x < XHistogramImage._width; x++) {
		if (!isFirstBlackPixel) {
			if (XHistogramImage(x, 0, 0) == 0) {
				isFirstBlackPixel = true;
				lastBlackPixel = x;
			}
		} else {
			if (x >= 1 && XHistogramImage(x, 0, 0) == 0 && XHistogramImage(x-1, 0, 0) == 255) {
				if (x - lastBlackPixel <= pixelThreshold) {
					for (int i = lastBlackPixel; i <= x; i++) {
						XHistogramImage(i, 0, 0) = 0;
						XHistogramImage(i, 0, 1) = 0;
						XHistogramImage(i, 0, 2) = 0;
					}
				}
				lastBlackPixel = x;
			}
			else if (x >= 1 && XHistogramImage(x, 0, 0) == 255 && XHistogramImage(x-1, 0, 0) == 0) {
				lastBlackPixel = x-1;
			}
		}
	}


	vector<int> keyPointsX = get_inflection_points_inX(XHistogramImage);
	// cout << "Key: " << keyPointsX.size() << endl;


	int color[] = { 255, 0, 0 };
	for (int i = 0; i < keyPointsX.size(); i++) {
		XHistogramImage.draw_line(keyPointsX[i], 0, keyPointsX[i], XHistogramImage._height-1, color);
	}
	if (cou) {
		XHistogramImage.display("Test1", false);
		// XHistogramImage.save("./MiddleResult/X_Segmentation.bmp");
		cou = false;
	}

	// 在X方向中进行分割
	vector<int> divide_in_X;
	divide_in_X.push_back(-1);
	if (keyPointsX.size() > 2) {
		for (int i = 1; i < keyPointsX.size()-1; i += 2) {
			int divide_line_point_x = (keyPointsX[i] + keyPointsX[i+1]) / 2;
			divide_in_X.push_back(divide_line_point_x);
		}
	}
	divide_in_X.push_back(XHistogramImage._width-1);

	// HistogramImage.display("XHistogramImage", false);
	return divide_in_X;
}

vector<int> Process::get_inflection_points_inX(const CImg<float>& XHistogramImage) {
	vector<int> inflection_X;
	vector<int> tempX;
	cimg_forX(XHistogramImage, x) {
		if (x >= 1) {
			if (XHistogramImage(x, 0, 0) == 0 && XHistogramImage(x-1, 0, 0) == 255) {
				tempX.push_back(x-1);
			}
			else if (XHistogramImage(x, 0, 0) == 255 && XHistogramImage(x-1, 0, 0) == 0) {
				tempX.push_back(x);
			}
		}
	}
	
	inflection_X.push_back(tempX[0]);
	for (int i = 2; i < tempX.size()-1; i += 2) {
		inflection_X.push_back(tempX[i-1]);
		inflection_X.push_back(tempX[i]);
	}

	inflection_X.push_back(tempX[tempX.size()-1]);
	/* 查找拐点 - 结束 */ 
	return inflection_X;
}

// 根据行分割的结果和相应列分割的拐点，生成数字图片的结果
vector<CImg<float>> Process::getRowImg(CImg<float>& row_image, vector<int> divideX) {
	vector<CImg<float>> result;
	for (int i = 1; i < divideX.size(); i++) {
		// 求得数字的宽度
		int rowItemWidth = divideX[i] - divideX[i-1];
		// 生成一张数字的图片
		CImg<float> rowItemImg = CImg<float>(rowItemWidth, row_image._height, 1, 1, 0);
		cimg_forXY(rowItemImg, x, y) {
			rowItemImg(x, y, 0) = row_image(x+divideX[i-1]+1, y, 0);
		}
		result.push_back(rowItemImg);
	}
	return result;
}

// 对分割后的数字做扩充
void Process::dilateImg(int barItemIndex) {
	CImg<float> DImg_XXY = CImg<float>(subImageSet[barItemIndex]._width, subImageSet[barItemIndex]._height,1, 1, 0);
	cimg_forXY(subImageSet[barItemIndex], x, y) {
		DImg_XXY(x, y) = dilate_process_xxy(subImageSet[barItemIndex], x, y);
	}

	CImg<float> DImg_XXY_2 = CImg<float>(DImg_XXY._width, DImg_XXY._height, 1, 1, 0);
	cimg_forXY(DImg_XXY, x, y) {
		DImg_XXY_2(x, y) = dilate_process_xxy(DImg_XXY, x, y);
	}

	CImg<float> DImg_XY = CImg<float>(DImg_XXY_2._width, DImg_XXY_2._height, 1, 1, 0);
	cimg_forXY(DImg_XXY_2, x, y) {
		DImg_XY(x, y) = dilate_process_xy(DImg_XXY_2, x, y);
	}

	cimg_forXY(subImageSet[barItemIndex], x, y) {
		subImageSet[barItemIndex](x, y, 0) = DImg_XY(x, y);
	}
}

// XY方向的正扩张
int Process::dilate_process_xy(const CImg<float>& Img, int x, int y) {
	int intensity = Img(x, y);
	if (intensity == 255) {
		for (int i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				if (x+i >= 0 && x+i < Img._width && y+j >= 0 && y+j < Img._height) {
					if (i != -1 && j != -1 || i != 1 && j != 1 || i != 1 && j != -1 || i != -1 && j != 1) {
						if (Img(x+i, y+j) == 0) {
							intensity = 0;
							break;
						}
					}
				}
			}
			if (intensity != 255) break;
		}
	}
	return intensity;
}


int Process::dilate_process_xxy(const CImg<float>& Img, int x, int y) {
	int intensity = Img(x, y, 0);
	if (intensity == 255) {
		int blackAccu = 0;
		for (int i = -1; i <= 1; i++) {
			if (y+i >= 0 && y+i < Img._height) {
				if (Img(x, y+i) == 0) {
					blackAccu++;
				}
			}
		}
		for (int i = -2; i <= 2; i++) {
			if (x+i >= 0 && x+i < Img._width) {
				if (x+i >= 0 && x+i < Img._width) {
					if (Img(x+i, y) == 0) {
						blackAccu--;
					}
				}
			}
		}
		if (blackAccu > 0) {
			intensity = 0;
		}
	}
	return intensity;
}

void Process::image_process(string filename) {
	for (int i = 0; i < subImageSet.size(); i++) {
		dilateImg(i);
		EightNeibourghLink(i);
	}
	saveSingleNumImg(filename);
}

void Process::EightNeibourghLink(int barItemIndex) {
	int new_pos_x, new_pos_y;
    int dx[8] = {1,1,0,-1,-1,-1,0,1},
        dy[8] = {0,1,1,1,0,-1,-1,-1};

	cimg_forXY(subImageSet[barItemIndex], x, y) {
		if (subImageSet[barItemIndex](x, y) == 0) {
			for (int i = 0; i < 8; i++) {
				new_pos_x = x + dx[i];
				new_pos_y = y + dy[i];
				subImageSet[barItemIndex](x, y) = 0;
			}
		}
	}
}

void Process::saveSingleNumImg(string filename) {
	// 先统计每张数字图像黑色像素个数平均值
	int totalBlacks = 0, numberCount = 0;

	int pos = 0;
	for (int i = 0; i < subImageSet.size(); i++) {
		string baseFile = filename;
		totalBlacks = subImageSet[i].size();
		numberCount = 0;
		cimg_forXY(subImageSet[i], x, y) {
			if (subImageSet[i](x, y) == 0) {
				numberCount++;
			}
		}
		double blackPixelRate = numberCount * 1.0 / totalBlacks;
		if (subImageSet[i].size() != 0 && blackPixelRate > 0.001 && blackPixelRate < 0.999) {
			
			// 先找到数字的包围盒
			int xMin = INT_MAX;
			int yMin = INT_MAX;
			int xMax = INT_MIN;
			int yMax = INT_MIN;

			cimg_forXY(subImageSet[i], x, y) {
				if (subImageSet[i](x, y) == 0) {
					xMin = xMin < x ? xMin : x;
					yMin = yMin < y ? yMin : y;
					xMax = xMax > x ? xMax : x;
					yMax = yMax > y ? yMax : y;
				}
			}

			int height = yMax - yMin;
			int width = xMax - xMin;
			CImg<float> singleNum(width, height, 1, 1, 255);

			for (int x = 0; x < width; x++) {
				for (int y = 0; y < height; y++) {
					singleNum(x, y) = subImageSet[i](x+xMin, y+yMin);
				}
			}

			CImg<float> finalResult(width+20, height+20, 1, 1, 255);
			for (int x = 10; x < width+10; x++) {
				for (int y = 10; y < height+10; y++) {
					finalResult(x, y) = singleNum(x-10, y-10);
				}
			}

			// 对单个数字图像做Y方向腐蚀操作
			char* buffer = new char[5];
			// string fileName = "./Segmentation/";
			_itoa(pos, buffer, 10);
			pos++;
			//itoa(i, buffer, 10);
			baseFile += buffer;
			baseFile += ".bmp";
			cimg_forXY(finalResult, x, y) {
				finalResult(x, y) = 255 - finalResult(x, y);
			}
			finalResult.save(baseFile.c_str());
			delete[] buffer;
		}
	}
}